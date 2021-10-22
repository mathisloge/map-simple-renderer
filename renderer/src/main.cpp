#include <filesystem>
#include <iostream>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/cairo/cairo_image_util.hpp>
#include <mapnik/cairo/cairo_renderer.hpp>
#include <mapnik/cairo_io.hpp>
#include <mapnik/color_factory.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/font_engine_freetype.hpp>
#include <mapnik/load_map.hpp>
#include <mapnik/map.hpp>
#include <spdlog/spdlog.h>
#include "bounded_buffer.hpp"
#include "version.hpp"
// https://github.com/openstreetmap/mapnik-stylesheets/blob/master/generate_tiles.py

constexpr double kPI = 3.141592653589793238462643383279502884197169399375105820974944592307816406;
constexpr double DEG_TO_RAD = kPI / 180.;
constexpr double RAD_TO_DEG = 180. / kPI;
namespace fs = std::filesystem;
static void render_tiles(const fs::path &tile_uri, int x, int y, int z);

static double minmax(double a, double b, double c)
{
    a = std::max(a, b);
    a = std::min(a, c);
    return a;
}
struct GoogleProjection
{
    std::vector<double> Bc;
    std::vector<double> Cc;
    std::vector<std::pair<double, double>> zc;
    std::vector<double> Ac;
    int c = 256;
    GoogleProjection(int levels = 18)
    {
        for (int d = 0; d <= levels; d++)
        {
            const double e = c / 2.;
            Bc.emplace_back(c / 360.0);
            Cc.emplace_back(c / (2. * kPI));
            zc.emplace_back(std::pair<double, double>{e, e});
            Ac.emplace_back(c);
            c *= 2;
        }
    }

    std::pair<double, double> fromLLtoPixel(const std::pair<double, double> &ll, int zoom) const
    {
        const auto &d = zc[zoom];
        const double e = round(d.first + ll.first * Bc[zoom]);
        const double f = minmax(std::sin(DEG_TO_RAD * ll.second), -0.9999, 0.9999);
        const double g = round(d.second + 0.5 * log((1 + f) / (1 - f)) * -Cc[zoom]);
        return {e, g};
    }

    std::pair<double, double> fromPixelToLL(const std::pair<double, double> &px, int zoom) const
    {
        const auto &e = zc[zoom];
        const auto f = (px.first - e.first) / Bc[zoom];
        const auto g = (px.second - e.second) / -Cc[zoom];
        const auto h = RAD_TO_DEG * (2 * atan(exp(g)) - 0.5 * kPI);
        return {f, h};
    }
};

struct RenderEntry
{
    fs::path tile_path;
    int x, y, z;
};
using Queue = bounded_buffer<RenderEntry>;
class RenderThread final
{
  public:
    RenderThread(int renderer_id, const fs::path &xml_file, Queue &queue, int w, int h)
        : renderer_id_{renderer_id}
        , map_file_{xml_file}
        , queue_{queue}
        , map_{w, h}
        , tileproj_{16}

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
        spdlog::info("setup renderer {}", renderer_id_);
        mapnik::load_map(map_, map_file_.string(), true);
        spdlog::info("renderer {} ready", renderer_id_);
        while (true)
        {
            RenderEntry e;
            queue_.pop_back(&e);
            try
            {
                render_tile(e.tile_path, e.x, e.y, e.z);
            }
            catch (const std::exception &ex)
            {
                spdlog::error("Error while rendering tile {} {} {}: {}", e.x, e.y, e.z, ex.what();)
            }
        }
        spdlog::info("renderer {} finished", renderer_id_);
    }

    void render_tile(const fs::path &tile_uri, int x, int y, int z)
    {
        spdlog::info("{}: rendering {} {} {}", renderer_id_, z, x, y);
        const std::pair<double, double> p0 = {x * 256., (y + 1) * 256.};
        const std::pair<double, double> p1 = {(x + 1.) * 256., y * 256.};
        const auto l0 = tileproj_.fromPixelToLL(p0, z);
        const auto l1 = tileproj_.fromPixelToLL(p1, z);

        mapnik::proj_transform proj_tr(std::string{"epsg:4326"}, map_.srs());
        double zd = z;
        double x0 = l0.first;
        double y0 = l0.second;
        double x1 = l1.first;
        double y1 = l1.second;
        proj_tr.forward(x0, y0, zd);
        proj_tr.forward(x1, y1, zd);

        const mapnik::box2d<double> bbox{x0, y0, x1, y1};
        map_.zoom_to_box(bbox);

        mapnik::image_rgba8 buf(map_.width(), map_.height());
        mapnik::agg_renderer<mapnik::image_rgba8> ren{map_, buf};
        ren.apply();
        save_to_file(buf, tile_uri.string(), "png");
    }

  private:
    const int renderer_id_;
    const fs::path map_file_;
    Queue &queue_;
    mapnik::Map map_;
    GoogleProjection tileproj_;
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
        , queue{100}
    {
        spdlog::info("creating {} render threads", num_threads);
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
        spdlog::info("All queued.");
    }

  private:
    void process(const BboxRenderJob &job)
    {
        const GoogleProjection gprj{job.max_zoom + 1};

        const std::pair<double, double> ll0{job.bbox[0], job.bbox[3]};
        const std::pair<double, double> ll1{job.bbox[2], job.bbox[1]};

        if (!fs::exists(tile_dir_))
            fs::create_directory(tile_dir_);

        spdlog::info("start rendering processing {}-{}", job.min_zoom, job.max_zoom);
        for (int z = job.min_zoom; z <= job.max_zoom; z++)
        {
            const double zpow = std::pow(2, z);
            const fs::path z_dir = tile_dir_ / std::to_string(z);
            if (!fs::exists(z_dir))
                fs::create_directory(z_dir);
            const auto px0 = gprj.fromLLtoPixel(ll0, z);
            const auto px1 = gprj.fromLLtoPixel(ll1, z);
            const int min_x = (px0.first / 256.0);
            const int max_x = (px1.first / 256.0) + 1;
            for (int x = min_x; x <= max_x; x++)
            {
                const int min_y = (px0.second / 256.0);
                const int max_y = (px1.second / 256.0) + 1;
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
                        spdlog::info("adding tile {} {} {}", z, x, y);
                        queue.push_front(RenderEntry{tile_uri, x, y, z});
                    }
                    else
                        spdlog::info("skipping tile {} {} {}", z, x, y);
                }
            }
        }
    }
};

int main(int argc, char const *argv[])
{
    const fs::path xml_file = "D:/dev/openstreetmap-carto/mapnik.xml";
    const fs::path out_dir = "D:/dev/map-simple-renderer/build/windows-64-default-release/server/websrc/tiles";
    mapnik::datasource_cache::instance().register_datasources(MAPNIK_PLUGINS_DIR);
    mapnik::freetype_engine::register_fonts("./fonts", true);
    constexpr int kNumThreads = 18;

    TileRenderer renderer{xml_file, out_dir, kNumThreads};
    const mapnik::box2d<double> world_bbox{-180.0, -90.0, 180.0, 90.0};
    renderer.render_tiles(world_bbox, 0, 5);

    const mapnik::box2d<double> bremen_bbox{8.60208, 53.17580, 9.11962, 52.93087};
    renderer.render_tiles(bremen_bbox, 6, 16);

    renderer.run();
#if 0
    mapnik::Map map{256, 256};
    mapnik::load_map(map, xml_file, true);

    map.zoom_to_box(mapnik::box2d<double>(653184, 6567768, 1366216, 7255286));

    mapnik::cairo_surface_ptr image_surface(cairo_image_surface_create(CAIRO_FORMAT_ARGB32, map.width(), map.height()),
                                            mapnik::cairo_surface_closer());
    double scale_factor = 1.0;
    mapnik::cairo_ptr image_context(mapnik::create_context(image_surface));
    mapnik::cairo_renderer<mapnik::cairo_ptr> png_render(map, image_context, scale_factor);
    png_render.apply();
    // we can now write to png with cairo functionality
    cairo_surface_write_to_png(&*image_surface, "cairo-demo.png");
    cairo_surface_finish(&*image_surface);

#endif
    return 0;
}
