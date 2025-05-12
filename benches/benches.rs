use criterion::{criterion_group, criterion_main, Criterion, BenchmarkId};
use dashmap::DashMap;
use itertools::Itertools;
use jagua_rs::entities::instances::instance_generic::InstanceGeneric;
use jagua_rs::entities::item::Item;
use jagua_rs::entities::problems::problem_generic::ProblemGeneric;
use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::geo_enums::AllowedRotation;
use jagua_rs::geometry::geo_traits::Shape;
use jagua_rs::geometry::primitives::simple_polygon::SimplePolygon;
use ordered_float::NotNan;
use rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator, ParallelIterator};
use sdr::discrete_item::Discretizable;
use sdr::discrete_line::DiscreteLine;
use std::cmp::Reverse;
use std::path::Path;

use jagua_rs::util::polygon_simplification::PolySimplConfig;
use sdr::io;
use sdr::sdr_config::SDRConfig;
use sdr::sdr_optimizer::SDROptimizer;
use sdr::sdr_parse::{ItemDiscrete, SdrParse};

fn bench_discrete_placement(c: &mut Criterion){
    let mut group = c.benchmark_group("discrete_placement_all_items");
    group.sample_size(10);

    let test_cases = vec![
        ("../assets/swim.json", 36.0, "swim"),
        ("../assets/shirts.json", 1.0, "shirts123"),
        ("../assets/trousers.json", 1.0, "trousers"),
        ("../assets/albano.json", 40.0, "albano"),
        ("../assets/dagli.json", 1.0, "dagli"),
        ("../assets/jakobs1.json", 1.0, "jakobs1"),
        ("../assets/jakobs2.json", 1.0, "jakobs2"),
    ];
    
    for (instance_path, resolution, name) in test_cases {
        // Setup the benchmark
        let config = SDRConfig::default();
        let path = Path::new(instance_path);
        let json_instance = io::read_json_instance(&path);
        let poly_simpl_config = PolySimplConfig::Disabled;
        let sdr_parser = SdrParse::new(poly_simpl_config, config.cde_config, true);
        let instance = sdr_parser.parse(&json_instance, resolution);
        
        // Benchmark discrete_placement for all items in sequence
        group.bench_with_input(
            BenchmarkId::new("all_items_seq", name),
            &(&instance, &config, resolution),
            |b, (instance, config, resolution)| {
                b.iter_with_setup(
                    || {
                        // Setup code - runs before each iteration but not timed
                        let optimizer = SDROptimizer::new((*instance).clone(), (*config).clone(), *resolution);
                        
                        // Sort items by descending diameter of convex hull
                        let sorted_item_indices = (0..optimizer.instance.instance.items().len())
                            .sorted_by_cached_key(|i| {
                                let item = &optimizer.instance.instance.items()[*i].0;
                                let ch = SimplePolygon::new(convex_hull_from_points(item.shape.points.clone()));
                                let ch_diam = NotNan::new(ch.diameter()).expect("convex hull diameter is NaN");
                                Reverse(ch_diam)
                            })
                            .collect_vec();
                        
                        (optimizer, sorted_item_indices)
                    },
                    |(mut optimizer, sorted_item_indices)| {
                        for item_index in sorted_item_indices {
                            let missing_items = &optimizer.problem.missing_item_qtys()[item_index];
                            let item = &optimizer.instance.instance.items()[item_index].0;
                            let mut rotation_lines = Vec::new();         
                                match &item.allowed_rotation {
                                    AllowedRotation::Discrete(angles) => {
                                        for rotation in angles {
                                            if let Some(ref_guard) = optimizer.instance.items.get(&ItemDiscrete { id: item_index, rotation: ordered_float::OrderedFloat(*rotation) }) {
                                                // Access the actual Vec<DiscreteLine> inside the Ref guard
                                                rotation_lines.push(ref_guard);
                                            }
                                        }
                                    },
                                    _ => {
                                        if let Some(ref_guard) = optimizer.instance.items.get(&ItemDiscrete { id: item_index, rotation: ordered_float::OrderedFloat(0.0) }) {
                                            // Access the actual Vec<DiscreteLine> inside the Ref guard
                                            rotation_lines.push(ref_guard);
                                        }
                                    },
                                }
                            SDROptimizer::discrete_placement(&optimizer.problem, &optimizer.instance.instance.item(item_index), &optimizer.resolution, &mut optimizer.discrete_strip, &rotation_lines, missing_items, &item_index);
                        }
                    },
                );
            },
        );
    }
    
    group.finish();
}

fn bench_discretization(c: &mut Criterion){
    let mut group = c.benchmark_group("discretization");
    group.sample_size(10);

    let test_cases = vec![
        ("../assets/swim.json", 36.0, "swim"),
        ("../assets/shirts.json", 1.0, "shirts123"),
        ("../assets/trousers.json", 1.0, "trousers"),
        ("../assets/albano.json", 40.0, "albano"),
        ("../assets/dagli.json", 1.0, "dagli"),
        ("../assets/jakobs1.json", 1.0, "jakobs1"),
        ("../assets/jakobs2.json", 1.0, "jakobs2"),
    ];
    
    for (instance_path, resolution, name) in test_cases {
        // Setup the benchmark
  
        
        // Benchmark discrete_placement for all items in sequence
        group.bench_with_input(
            BenchmarkId::new("discretization", name),
            &(resolution),
            |b, resolution| {
                b.iter_with_setup(
                    || {
                        // Setup code - runs before each iteration but not timed
                        let config = SDRConfig::default();
                        let path = Path::new(instance_path);
                        let json_instance = io::read_json_instance(&path);
                        let poly_simpl_config = PolySimplConfig::Disabled;
                        let sdr_parser = SdrParse::new(poly_simpl_config, config.cde_config, true);
                        let items: Vec<(Item, usize)> = json_instance
                            .items
                            .par_iter()
                            .enumerate()
                            .map(|(item_id, json_item)| sdr_parser.parse_item(json_item, item_id))
                            .collect();
                        
                        
                        items
                    },
                    |it| {
                        let items: DashMap<ItemDiscrete, Vec<DiscreteLine>> = DashMap::new();
                        //let start_time = Instant::now();
                        // Iterate through each item in the instance
                        it.par_iter().for_each(|item| {
                            // let item_time = Instant::now();
                            // Create an ItemDiscrete for this item
                            match &item.0.allowed_rotation {
                                AllowedRotation::Discrete(angles) => {
                                    for rotation in angles {
                                        let item_discrete = ItemDiscrete::new(&item.0.id, rotation);
                                        let discrete_lines = item.0.discretize_shape(*resolution, *rotation);
                                        items.insert(item_discrete, discrete_lines);
                                    }
                                },
                                _ => {
                                    let item_discrete = ItemDiscrete::new(&item.0.id, &0.0);   
                                    let discrete_lines = item.0.discretize_shape(*resolution, 0.0); 
                                    items.insert(item_discrete, discrete_lines);
                                },
                            }

                        });
                    },
                );
            },
        );
    }
}

criterion_group!(
    benches, 
    bench_discrete_placement,
    bench_discretization,
);

criterion_main!(benches);