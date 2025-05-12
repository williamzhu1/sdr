
use sdr::sdr_parse::SdrParse;
use sdr::sdr_config::SDRConfig;
use sdr::sdr_optimizer::SDROptimizer;
use sdr::io::{self};
use std::path::Path as OtherPath;
use jagua_rs::util::polygon_simplification::PolySimplConfig;


fn main() {
    let instance = OtherPath::new("../assets/shirts.json");
    // parse the instance
    let mut config = SDRConfig::default();
    config.n_samples = 100;
    let json_instance = io::read_json_instance(&instance);
    let poly_simpl_config = match config.poly_simpl_tolerance {
        Some(tolerance) => PolySimplConfig::Enabled { tolerance },
        None => PolySimplConfig::Disabled,
    };

    let sdr_parser = SdrParse::new(poly_simpl_config, config.cde_config, true);
    let instance = sdr_parser.parse(&json_instance, 1.0);
    let mut optimizer = SDROptimizer::new(instance, config, 1.0);
    let _sol = optimizer.solve();
}
