#[cfg(test)]
mod test_placement {
    use jagua_rs::entities::instances::instance_generic::InstanceGeneric;
    use jagua_rs::geometry::geo_traits::Transformable;
    use jagua_rs::geometry::transformation::Transformation;
    use jagua_rs::io::parser::Parser;
    use sdr::io::svg_export::simple_polygon_data;
    use sdr::sdr_config::SDRConfig;
    use sdr::sdr_parse::SdrParse;
    use test_case::test_case;
    use sdr::discrete_item::Discretizable;
    use sdr::{discrete_line, io};
    use svg::node::element::{Path, Line};
    use svg::Document;
    use std::fs::File;
    use std::io::Write;
    use std::path::Path as OtherPath;
    use jagua_rs::util::polygon_simplification::PolySimplConfig;
    use std::time::{Instant, Duration};
    use simplelog::{Config, LevelFilter, WriteLogger};
    use chrono::{DateTime, Local};

    #[test_case("../assets/shirt2.json", 1.0; "shirt2")]
    #[test_case("../assets/swim.json", 50.0; "swim")]
    #[test_case("../assets/shirts.json", 1.1; "shirtsd")]
    #[test_case("../assets/trousers.json", 0.2; "trousers")]
    #[test_case("../assets/mao.json", 50.0; "mao")]
    fn test_placement(instance_path: &str, resolution: f32) {
        // Get current timestamp
        let now: DateTime<Local> = Local::now();
        let timestamp = now.format("%Y-%m-%d_%H-%M-%S").to_string();

        let log_filename = format!("logs/log_{}.log", timestamp);
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
        log::info!("instance: {:?}", instance.items);
        for (i, item) in instance.instance.items().iter().enumerate() {
            let moved_shape = item.0.shape.transform_clone(&item.0.move_to_first_quadrant(0.0));
            let rotated_shape = item.0.shape.transform_clone(&item.0.move_to_first_quadrant(3.1415927));
            let discretized_shape = item.0.discretize_shape(resolution, 0.0);
            let rotatedd_shape = item.0.discretize_shape(resolution, 3.1415927);
            log::info!("item segments: {:?}", discretized_shape);
            let polygon_data = simple_polygon_data(&moved_shape);
            let rotated_data = simple_polygon_data(&rotated_shape);
            // Obtain the bounding box from the polygon's shape
            let bbox = &moved_shape.bbox;
            let rotated_bbox = &rotated_shape.bbox;
            let width = bbox.width();
            let height = bbox.height();
            let rotated_width = rotated_bbox.width();
            let rotated_height = rotated_bbox.height();

            // Add some margin (e.g., 10%)
            let margin_x = width * 0.1;
            let margin_y = height * 0.1;

            let rotated_x = rotated_width * 0.1;
            let rotated_y = rotated_height * 0.1;


            // Calculate the viewBox coordinates
            let view_x = bbox.x_min - margin_x;
            let view_y = bbox.y_min - margin_y;
            let view_w = width + 2.0 * margin_x;
            let view_h = height + 2.0 * margin_y;

            let rotate_x = rotated_bbox.x_min - rotated_x;
            let rotate_y = rotated_bbox.y_min - rotated_y;
            let rotate_w = rotated_width + 2.0 * rotated_x;
            let rotate_h = rotated_height + 2.0 * rotated_y;

            // Create polygon path
            let polygon_path = Path::new()
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", 0.05)
                .set("d", polygon_data);
            let rotated_path = Path::new()
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", 0.05)
                .set("d", rotated_data);

            // Use the dynamically computed viewBox here instead of a fixed one
            let mut document = Document::new()
                .set("viewBox", (view_x, view_y, view_w, view_h))
                // The width and height of the rendered SVG can remain fixed or be adjusted.
                // Using the same width/height in pixels is fine; SVG will scale to viewBox.
                .set("width", 800)
                .set("height", 600)
                .add(polygon_path);

            let mut rotated_document = Document::new()
                .set("viewBox", (rotate_x, rotate_y, rotate_w, rotate_h))
                // The width and height of the rendered SVG can remain fixed or be adjusted.
                // Using the same width/height in pixels is fine; SVG will scale to viewBox.
                .set("width", 800)
                .set("height", 600)
                .add(rotated_path);

            for segments in discretized_shape {
                for occupied in segments.occupied {
                    let line = Line::new()
                        .set("x1", segments.id as f32 * resolution)
                        .set("y1", -occupied.start)
                        .set("x2", segments.id as f32 * resolution)
                        .set("y2", -occupied.end)
                        .set("stroke", "red")
                        .set("stroke-width", 0.05);
                    document = document.add(line);
                }
            }

            for segments in rotatedd_shape {
                for occupied in segments.occupied {
                    let line = Line::new()
                        .set("x1", segments.id as f32 * resolution)
                        .set("y1", -occupied.start)
                        .set("x2", segments.id as f32 * resolution)
                        .set("y2", -occupied.end)
                        .set("stroke", "red")
                        .set("stroke-width", 0.05);
                    rotated_document = rotated_document.add(line);
                }
            }

            // Save to file
            let filename = format!("discretized_shape_{}.svg", i);
            let mut file = File::create(&filename).expect("Unable to create file");
            file.write_all(document.to_string().as_bytes()).expect("Unable to write SVG data");

            let rotated_filename = format!("discretized_shape_rotated_{}.svg", i);
            let mut rotated_file = File::create(&rotated_filename).expect("Unable to create file");
            rotated_file.write_all(rotated_document.to_string().as_bytes()).expect("Unable to write SVG data");

            println!("SVG generated and saved as discretized_shape_{}.svg", i);
        }

    }
}
