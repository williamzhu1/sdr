#[cfg(test)]
mod test_placement {
    use jagua_rs::fsize;
    use sdr::io::svg_util::{SvgDrawOptions, SvgLayoutTheme};
    use sdr::sdr_parse::SdrParse;
    use sdr::{io::layout_to_svg::s_layout_to_svg, sdr_config::SDRConfig};
    use sdr::sdr_optimizer::SDROptimizer;
    use test_case::test_case;
    use sdr::io::{self};
    use std::fs::File;
    use std::io::Write;
    use std::path::Path as OtherPath;
    use jagua_rs::util::polygon_simplification::PolySimplConfig;
    use simplelog::{Config, LevelFilter, WriteLogger};
    use chrono::{DateTime, Local};

    #[test_case("../assets/shirt2.json", 1.0; "shirt2")]
    #[test_case("../assets/swim.json", 36.0; "swim")]
    #[test_case("../assets/shirts.json", 1.0; "shirts123")]
    #[test_case("../assets/shirtest.json", 1.0; "shirtest")]
    #[test_case("../assets/trousers.json", 1.0; "trousers")]    
    #[test_case("../assets/albano.json", 40.0; "albano")]
    #[test_case("../assets/dagli.json", 1.0; "dagli")]
    #[test_case("../assets/jakobs1.json", 1.0; "jakobs1")]
    #[test_case("../assets/jakobs2.json", 1.0; "jakobs2")]
    fn test_placement(instance_path: &str, resolution: fsize) {

        let now: DateTime<Local> = Local::now();
        let timestamp = now.format("%Y-%m-%d_%H-%M-%S").to_string();

        let log_filename = format!("logs/place_{}.log", timestamp);
        let log_file = File::create(log_filename);
        let _ = WriteLogger::init(LevelFilter::Info, Config::default(), log_file.unwrap());

        let instance = OtherPath::new(instance_path);
        // parse the instance
        let mut config = SDRConfig::default();
        config.n_samples = 100;
        let json_instance = io::read_json_instance(&instance);
        let poly_simpl_config = match config.poly_simpl_tolerance {
            Some(tolerance) => PolySimplConfig::Enabled { tolerance },
            None => PolySimplConfig::Disabled,
        };

        let sdr_parser = SdrParse::new(poly_simpl_config, config.cde_config, true);
        let instance = sdr_parser.parse(&json_instance, resolution);
        let mut optimizer = SDROptimizer::new(instance, config, resolution);
        let sol = optimizer.solve();
        log::info!("discreteStrip: {:?}", optimizer.discrete_strip);
        log::info!("target items: {:?}", sol.target_item_qtys);
        let svg = s_layout_to_svg(&sol.layout_snapshots[0], &optimizer.instance.instance, SvgDrawOptions {
            theme: SvgLayoutTheme::default(), // You need to define or load a theme here
            surrogate: false, // Optional: whether to include surrogates in the SVG
            quadtree:false,  // Optional: whether to include quadtree
            haz_prox_grid: false, // Optional: whether to include hazard proximity grid
        });
        let mut file = File::create("solution.svg").expect("Unable to create file");
        file.write_all(svg.to_string().as_bytes()).expect("Unable to write SVG data");
    }
    
}
