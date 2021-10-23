#include <chrono>
#include <filesystem>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/color_factory.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/font_engine_freetype.hpp>
#include <mapnik/load_map.hpp>
#include <mapnik/map.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include "bounded_buffer.hpp"
#include "version.hpp"
// https://github.com/openstreetmap/mapnik-stylesheets/blob/master/generate_tiles.py

constexpr double kPI = 3.141592653589793238462643383279502884197169399375105820974944592307816406;
constexpr double DEG_TO_RAD = kPI / 180.;
constexpr double RAD_TO_DEG = 180. / kPI;
namespace fs = std::filesystem;

static int long2tilex(double lon, int z)
{
    return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
}

static int lat2tiley(double lat, int z)
{
    const double latrad = lat * DEG_TO_RAD;
    return (int)(floor((1.0 - asinh(tan(latrad)) / kPI) / 2.0 * (1 << z)));
}

static double tilex2long(int x, int z)
{
    return x / (double)(1 << z) * 360.0 - 180.;
}

static double tiley2lat(int y, int z)
{
    double n = kPI - 2.0 * kPI * y / (double)(1 << z);
    return RAD_TO_DEG * atan(0.5 * (exp(n) - exp(-n)));
}

struct Options
{};
struct RenderEntry
{
    fs::path tile_path;
    int x, y, z;
};
using Queue = bounded_buffer<RenderEntry>;
class RenderThread final
{
    using MyRenderer = mapnik::agg_renderer<mapnik::image_rgba8>;

  public:
    RenderThread(int renderer_id, const fs::path &xml_file, Queue &queue, int w, int h)
        : renderer_id_{renderer_id}
        , map_file_{xml_file}
        , queue_{queue}

    {
        render_thread_ = std::thread{std::bind(&RenderThread::render_thread, this)};
    }

    ~RenderThread()
    {
        if (render_thread_.joinable())
            render_thread_.join();
    }

    void render_thread()
    {
        auto logger = spdlog::get("console");
        logger->info("setup renderer {}", renderer_id_);
        mapnik::Map map;
        mapnik::load_map(map, map_file_.string(), true);
        logger->info("renderer {} ready", renderer_id_);

        while (true)
        {
            RenderEntry e;
            queue_.pop_back(&e);
            bool retry = true;
            while (retry)
            {
                try
                {
                    render_tile(map, e.tile_path, e.x, e.y, e.z);
                    retry = false;
                }
                catch (const std::exception &ex)
                {
                    logger->error("Error while rendering tile {} {} {}: {}", e.z, e.x, e.y, ex.what());
                }
            }
        }
        logger->info("renderer {} finished", renderer_id_);
    }

    void render_tile(mapnik::Map &map, const fs::path &tile_uri, int x, int y, int z)
    {
        auto logger = spdlog::get("console");
        const std::pair<int, int> p0 = {x, y + 1};
        const std::pair<int, int> p1 = {x + 1, y};

        double zd = z;
        double x0 = tilex2long(p0.first, z);
        double y0 = tiley2lat(p0.second, z);
        double x1 = tilex2long(p1.first, z);
        double y1 = tiley2lat(p1.second, z);
        logger->info("renderer {}: bb({} {} {}) {} {} {} {}", renderer_id_, z, x, y, x0, y0, x1, y1);
        {
            const mapnik::proj_transform proj_tr{std::string{"epsg:4326"}, map.srs()};
            proj_tr.forward(x0, y0, zd);
            proj_tr.forward(x1, y1, zd);
        }

        const mapnik::box2d<double> bbox{x0, y0, x1, y1};
        map.zoom_to_box(bbox);
        if (map.buffer_size() < 128)
            map.set_buffer_size(128);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        mapnik::image_rgba8 buf(map.width(), map.height());
        {
            mapnik::agg_renderer<mapnik::image_rgba8> ren{map, buf};
            ren.apply();
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        logger->info("renderer {} applied: {}",
                     renderer_id_,
                     std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
        save_to_file(buf, tile_uri.string(), "png");
        logger->info("renderer {}: processed {} {} {}", renderer_id_, z, x, y);
    }

  private:
    const int renderer_id_;
    const fs::path map_file_;
    Queue &queue_;
    std::thread render_thread_;
};
struct BboxRenderJob
{
    const mapnik::box2d<double> bbox;
    int min_zoom;
    int max_zoom;
};
class TileRenderer final
{
    Queue queue;
    std::vector<std::unique_ptr<RenderThread>> renderers;
    const fs::path tile_dir_;
    std::vector<BboxRenderJob> render_jobs_;

  public:
    TileRenderer(const fs::path &xml_file, const fs::path &tile_dir, int num_threads)
        : tile_dir_{tile_dir}
        , queue{num_threads * 2}
    {
        spdlog::get("console")->info("creating {} render threads", num_threads);
        for (int i = 0; i < num_threads; i++)
            renderers.emplace_back(std::make_unique<RenderThread>(i, xml_file, queue, 256, 256));
    }

    ~TileRenderer()
    {
        renderers.clear();
    }

    void render_tiles(const mapnik::box2d<double> bbox, int min_zoom, int max_zoom)
    {
        render_jobs_.emplace_back(BboxRenderJob{bbox, min_zoom, max_zoom});
    }

    void run()
    {
        for (const auto &job : render_jobs_)
            process(job);
        spdlog::get("console")->info("All queued.");
    }

  private:
    void process(const BboxRenderJob &job)
    {
        auto logger = spdlog::get("console");
        if (!fs::exists(tile_dir_))
            fs::create_directory(tile_dir_);

        logger->info("start rendering processing {}-{}", job.min_zoom, job.max_zoom);
        for (int z = job.min_zoom; z <= job.max_zoom; z++)
        {
            const double zpow = std::pow(2, z);
            const fs::path z_dir = tile_dir_ / std::to_string(z);
            if (!fs::exists(z_dir))
                fs::create_directory(z_dir);
            const int min_x = long2tilex(job.bbox.minx(), z);
            const int max_x = long2tilex(job.bbox.maxx(), z);
            for (int x = min_x; x <= max_x; x++)
            {
                const int min_y = lat2tiley(job.bbox.maxy(), z);
                const int max_y = lat2tiley(job.bbox.miny(), z);
                if ((x < 0) || (x >= zpow))
                    continue;
                const fs::path x_dir = z_dir / std::to_string(x);
                if (!fs::exists(x_dir))
                    fs::create_directory(x_dir);

                for (int y = min_y; y <= max_y; y++)
                {
                    if ((y < 0) || (y >= zpow))
                        continue;
                    const fs::path tile_uri{tile_dir_ / std::to_string(z) / std::to_string(x) /
                                            (std::to_string(y) + ".png")};
                    if (!fs::exists(tile_uri))
                    {
                        logger->debug("adding tile {} {} {}", z, x, y);
                        queue.push_front(RenderEntry{tile_uri, x, y, z});
                    }
                    else
                        logger->debug("skipping tile {} {} {}", z, x, y);
                }
            }
        }
    }
};

int main(int argc, char const *argv[])
{
    auto console = spdlog::stdout_color_mt("console");
    console->set_pattern("[con](%t) [%^%l%$](%H:%M:%S.%e) %v");
    const fs::path xml_file = "D:/dev/openstreetmap-carto/mapnik.xml";
    const fs::path out_dir = "D:/dev/map-simple-renderer/build/windows-64-default-release/server/websrc/tiles";
    mapnik::datasource_cache::instance().register_datasources(renderer::kMAPNIK_PLUGINS_DIR);
    mapnik::freetype_engine::register_fonts("./fonts", true);
    constexpr int kNumThreads = 8;

    TileRenderer renderer{xml_file, out_dir, kNumThreads};
    const mapnik::box2d<double> world_bbox{-180.0, -90.0, 180.0, 90.0};
    renderer.render_tiles(world_bbox, 0, 1);

    const mapnik::box2d<double> germany_bbox{6.0, 49.0, 11.0, 56.0};
    renderer.render_tiles(germany_bbox, 2, 5);

    const mapnik::box2d<double> north_west_germany{6, 50, 10, 54};
    renderer.render_tiles(north_west_germany, 6, 16);

    renderer.run();
    return 0;
}
