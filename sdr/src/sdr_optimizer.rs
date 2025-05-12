use std::cmp::Reverse;
use std::time::Instant;

use itertools::Itertools;
use jagua_rs::fsize;
use jagua_rs::geometry::d_transformation::DTransformation;
use jagua_rs::geometry::geo_enums::AllowedRotation;
use ordered_float::NotNan;


use jagua_rs::entities::instances::instance::Instance;
use jagua_rs::entities::instances::instance_generic::InstanceGeneric;
use jagua_rs::entities::item::Item;
use jagua_rs::entities::placing_option::PlacingOption;

use jagua_rs::entities::problems::problem::Problem;
use jagua_rs::entities::problems::problem_generic::{ProblemGeneric, STRIP_LAYOUT_IDX};
use jagua_rs::entities::problems::strip_packing::SPProblem;
use jagua_rs::entities::solution::Solution;
use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::geo_traits::Shape;
use jagua_rs::geometry::primitives::simple_polygon::SimplePolygon;


use crate::discrete_item::Discretizable;

use crate::discrete_line::{get_overlaps_count, DiscreteLine, DiscreteStrip};
use crate::sdr_config::SDRConfig;
use crate::sdr_parse::{ItemDiscrete, SdrInstance};


pub const ITEM_LIMIT: usize = usize::MAX;

pub struct SDROptimizer{
    pub instance: SdrInstance,
    pub problem: Problem,
    pub config: SDRConfig,
    pub resolution: fsize,
    pub discrete_strip: DiscreteStrip,
    pub transforms: Vec<DTransformation>,
}

impl SDROptimizer {
    pub fn new(instance: SdrInstance, config: SDRConfig, resolution: fsize) -> Self{
        let problem = match instance.instance.clone() {
            Instance::SP(spi) => {
                let strip_width = instance.instance.item_area() * 2.0 / spi.strip_height; //initiate with 50% usage            
                SPProblem::new(spi.clone(), strip_width, config.cde_config).into()
            }
            _ => panic!("SDROptimizer only supports Strip Packing problems (SPProblem)")
        };
        let width = match &problem {
            Problem::SP(sp_problem) => sp_problem.strip_width(),
            _ => panic!("SDROptimizer only supports Strip Packing problems")
        };
        let discrete_strip = DiscreteStrip { lines: Self::generate_discrete_lines(width, resolution)};
        let transforms = Vec::new();
        
        Self { instance, problem, config, resolution, discrete_strip, transforms}
    }

    pub fn generate_discrete_lines(strip_width: fsize, resolution: fsize) -> Vec<DiscreteLine> {
        let num_lines = (strip_width / resolution).ceil() as u32;
        
        (0..num_lines)
            .map(|_i| {
                DiscreteLine::new()
            })
            .collect()
    }

    pub fn solve(&mut self) -> Solution {
        //sort the items by descending diameter of convex hull 
        let mut opts: Vec<PlacingOption> = Vec::new();
        let sorted_item_indices = (0..self.instance.instance.items().len())
            .sorted_by_cached_key(|i| {
                let item = &self.instance.instance.items()[*i].0;
                let ch = SimplePolygon::new(convex_hull_from_points(item.shape.points.clone()));
                let ch_diam = NotNan::new(ch.diameter()).expect("convex hull diameter is NaN");
                Reverse(ch_diam)
            })
            .collect_vec();
        //log::info!("sorted {:?}", sorted_item_indices);

        let start_time = Instant::now();
        for item_index in &sorted_item_indices {
            let item = &self.instance.instance.items()[*item_index].0;
            let missing_items = &self.problem.missing_item_qtys()[*item_index];
            let mut rotation_lines = Vec::new();         
                match &item.allowed_rotation {
                    AllowedRotation::Discrete(angles) => {
                        for rotation in angles {
                            if let Some(ref_guard) = self.instance.items.get(&ItemDiscrete { id: *item_index, rotation: ordered_float::OrderedFloat(*rotation) }) {
                                // Access the actual Vec<DiscreteLine> inside the Ref guard
                                rotation_lines.push(ref_guard);
                            }
                        }
                    },
                    _ => {
                        if let Some(ref_guard) = self.instance.items.get(&ItemDiscrete { id: *item_index, rotation: ordered_float::OrderedFloat(0.0) }) {
                            // Access the actual Vec<DiscreteLine> inside the Ref guard
                            rotation_lines.push(ref_guard);
                        }
                    },
                }
                let placements = Self::discrete_placement(&self.problem, item, &self.resolution, &mut self.discrete_strip, &rotation_lines, missing_items, item_index);
                for placement in placements{
                    opts.insert(0, placement);
                }
        }
        println!("Total overlaps calls {}", get_overlaps_count());
        println!("Runningtook {:?} ", start_time.elapsed());

        match &mut self.problem {
            Problem::BP(_) => {}
            Problem::SP(sp_problem) => {
                self.discrete_strip.trim_after_last_occupied();
                sp_problem.modify_strip_in_back((self.discrete_strip.lines.len() - 1) as fsize * self.resolution);
                log::info!(
                    "[LBF] fitted strip width to {:.3}",
                    sp_problem.strip_width()
                )
            }
        }

        'outer: for item_index in sorted_item_indices {
            while self.problem.missing_item_qtys()[item_index] > 0{
                self.problem.place_item(opts.pop().unwrap());
                #[allow(clippy::absurd_extreme_comparisons)]
                if self.problem.placed_item_qtys().sum::<usize>() >= ITEM_LIMIT {
                    break 'outer;
                }
            }

        }
        let solution= self.problem.create_solution(None);
        
        match &mut self.problem {
            Problem::BP(_) => {}
            Problem::SP(sp_problem) => {
                println!("Strip Width {:?}, Usage {:?} % ", sp_problem.occupied_width(), sp_problem.layout.usage() * 100.00);
            }
        }
        
        solution
    }

    pub fn discrete_placement(
        problem: &Problem,
        item: &Item,
        resolution: &fsize,
        discrete_strip: &mut DiscreteStrip,
        rotation_lines: &Vec<&Vec<DiscreteLine>>,
        amount: &isize,
        item_index: &usize,
    ) -> Vec<PlacingOption>{
        let mut placements: Vec<PlacingOption> = Vec::new();
        match problem {
            Problem::BP(_) => {}
            Problem::SP(sp_problem) => {
                let results = discrete_strip.try_fit_segments(
                    rotation_lines, 
                    &sp_problem.strip_height(), 
                    resolution,
                    amount,
                    item_index
                );
                for result in results{
                    let translation_1 = result.0.decompose().translation; 

                    let new_transformation = match &item.allowed_rotation {
                        AllowedRotation::Discrete(angles) => {
                            let angle = angles[result.1];
                            let translation_2 = item.move_to_first_quadrant(angle).decompose().translation; 
                            let added_translation = (
                                translation_1.0.into_inner() + translation_2.0.into_inner(), 
                                translation_1.1.into_inner() + translation_2.1.into_inner(), 
                            );
                            DTransformation::new(
                                angle,            // Use the selected angle
                                added_translation,              // Use the added translation
                            )
                        },
                        _ => {
                            
                            let translation_2 = item.move_to_first_quadrant(0.0).decompose().translation; 
                            let added_translation = (
                                translation_1.0.into_inner() + translation_2.0.into_inner(), 
                                translation_1.1.into_inner() + translation_2.1.into_inner(), 
                            );
                            DTransformation::new(
                                0.0,            // Default rotation
                                added_translation,              // Use the added translation
                            )
                        },
                    };
                    // println!("pushing {}, {:?}", item.id, new_transformation);
                    placements.push(PlacingOption { layout_idx: STRIP_LAYOUT_IDX, item_id: item.id, d_transf: new_transformation });
                }
            }
        }
        placements
    }

}
