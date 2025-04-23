#[cfg(test)]
mod test_placement {
    use jagua_rs::io::parser::Parser;
    use sdr::io::svg_util::{SvgDrawOptions, SvgLayoutTheme};
    use sdr::{io::layout_to_svg::s_layout_to_svg, sdr_config::SDRConfig};
    use sdr::sdr_optimizer::SDROptimizer;
    use test_case::test_case;
    use sdr::io::{self, svg_export};
    use svg::Document;
    use svg::node::element::{Path, Line};
    use std::fs::File;
    use std::io::Write;
    use std::path::Path as OtherPath;
    use jagua_rs::util::polygon_simplification::PolySimplConfig;
    use std::time::{Instant, Duration};
    use log::{info, warn, error, debug, trace};
    use simplelog::{Config, LevelFilter, WriteLogger};

    #[test_case("../assets/shirt2.json", 1.0; "shirt2")]
    //#[test_case("../assets/swim.json", 1.0; "swim")]
    #[test_case("../assets/shirts.json", 1.0; "shirts")]
    //#[test_case("../assets/trousers.json", 1.0; "trousers")]
    //#[test_case("../assets/mao.json", 1.0; "mao")]
    fn test_placement(instance_path: &str, resolution: f32) {

        let log_file = File::create("logs/my_app.log");
        WriteLogger::init(LevelFilter::Info, Config::default(), log_file.unwrap());

        let instance = OtherPath::new(instance_path);
        // parse the instance
        let mut config = SDRConfig::default();
        config.n_samples = 100;
        let json_instance = io::read_json_instance(&instance);
        let poly_simpl_config = match config.poly_simpl_tolerance {
            Some(tolerance) => PolySimplConfig::Enabled { tolerance },
            None => PolySimplConfig::Disabled,
        };

        let parser = Parser::new(poly_simpl_config, config.cde_config, true);
        let instance = parser.parse(&json_instance);
        let mut optimizer = SDROptimizer::new(instance.clone(), config, resolution);
        let sol = optimizer.solve();
        log::info!("items: {:?}", sol.placed_item_qtys);
        log::info!("target items: {:?}", sol.target_item_qtys);
        let svg = s_layout_to_svg(&sol.layout_snapshots[0], &instance, SvgDrawOptions {
            theme: SvgLayoutTheme::default(), // You need to define or load a theme here
            surrogate: false, // Optional: whether to include surrogates in the SVG
            quadtree:false,  // Optional: whether to include quadtree
            haz_prox_grid: false, // Optional: whether to include hazard proximity grid
        });
        let mut file = File::create("solution.svg").expect("Unable to create file");
        file.write_all(svg.to_string().as_bytes()).expect("Unable to write SVG data");
    }
    
}
