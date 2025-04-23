use std::cmp::{Ordering, Reverse};
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

use std::time::Instant;

use itertools::Itertools;
use jagua_rs::geometry::d_transformation::DTransformation;
use jagua_rs::geometry::transformation::Transformation;
use log::{debug, info};
use ordered_float::NotNan;


use jagua_rs::entities::instances::instance::Instance;
use jagua_rs::entities::instances::instance_generic::InstanceGeneric;
use jagua_rs::entities::item::Item;
use jagua_rs::entities::placing_option::PlacingOption;

use jagua_rs::entities::problems::problem::{self, Problem};
use jagua_rs::entities::problems::problem_generic::{LayoutIndex, ProblemGeneric, STRIP_LAYOUT_IDX};
use jagua_rs::entities::problems::strip_packing::SPProblem;
use jagua_rs::entities::solution::{self, Solution};
use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::geo_traits::{Shape, TransformableFrom};
use jagua_rs::geometry::primitives::simple_polygon::SimplePolygon;


use crate::discrete_item::Discretizable;

use crate::discrete_line::{self, DiscreteLine, DiscreteStrip};
use crate::sdr_config::{self, SDRConfig};


pub const ITEM_LIMIT: usize = usize::MAX;

pub struct SDROptimizer{
    pub instance: Instance,
    pub problem: Problem,
    pub config: SDRConfig,
    pub resolution: f32,
    pub discrete_strip: DiscreteStrip,
    pub transforms: Vec<DTransformation>,
}

impl SDROptimizer {
    pub fn new(instance: Instance, config: SDRConfig, resolution: f32) -> Self{
        let problem = match instance.clone() {
            Instance::SP(spi) => {
                let strip_width = instance.item_area() * 2.0 / spi.strip_height; //initiate with 50% usage            
                SPProblem::new(spi.clone(), strip_width, config.cde_config).into()
            }
            _ => panic!("SDROptimizer only supports Strip Packing problems (SPProblem)")
        };
        let width = match &problem {
            Problem::SP(sp_problem) => sp_problem.strip_width(),
            _ => panic!("SDROptimizer only supports Strip Packing problems")
        };
        let discrete_strip = DiscreteStrip { lines: Self::generate_discrete_lines(width, resolution) };
        let transforms = Vec::new();
        
        Self { instance, problem, config, resolution, discrete_strip, transforms}
    }

    pub fn generate_discrete_lines(strip_width: f32, resolution: f32) -> Vec<DiscreteLine> {
        let num_lines = (strip_width / resolution).ceil() as u32;
        
        (0..num_lines)
            .map(|i| {
                DiscreteLine::new(i as usize)
            })
            .collect()
    }

    pub fn solve(&mut self) -> Solution {
        //sort the items by descending diameter of convex hull
        let sorted_item_indices = (0..self.instance.items().len())
            .sorted_by_cached_key(|i| {
                let item = &self.instance.items()[*i].0;
                let ch = SimplePolygon::new(convex_hull_from_points(item.shape.points.clone()));
                let ch_diam = NotNan::new(ch.diameter()).expect("convex hull diameter is NaN");
                Reverse(ch_diam)
            })
            .collect_vec();
        log::info!("sorted {:?}", sorted_item_indices);
        let now = Instant::now();
        'outer: for item_index in sorted_item_indices {
            let item = &self.instance.items()[item_index].0;
            //place all items of this type
            while self.problem.missing_item_qtys()[item_index] > 0 {
                //find a position and insert it
                match &mut Self::discrete_placement(
                    &self.problem,
                    item,
                    &self.resolution,
                    &mut self.discrete_strip,
                ) {
                    Some(i_opt) => {
                        let l_index = self.problem.place_item(*i_opt);
                        log::info!(
                            "[SDR] placing item {}/{} with id {} at [{}] in Layout {:?}",
                            self.problem.placed_item_qtys().sum::<usize>(),
                            self.instance.total_item_qty(),
                            i_opt.item_id,
                            i_opt.d_transf,
                            l_index
                        );
                        #[allow(clippy::absurd_extreme_comparisons)]
                        if self.problem.placed_item_qtys().sum::<usize>() >= ITEM_LIMIT {
                            break 'outer;
                        }
                    }
                    None => match &mut self.problem {
                        Problem::BP(_) => break,
                        Problem::SP(sp_problem) => {
                            let new_width = sp_problem.strip_width() * 1.1;
                            // Calculate how many new lines we need to add
                            let current_num_lines = self.discrete_strip.lines.len() as f32;
                            let new_num_lines = (new_width / self.resolution).ceil();
                            
                            // Only add the missing lines (new ones beyond the current number of lines)
                            let lines_to_add = (new_num_lines - current_num_lines).ceil() as u32;

                            // Append the missing lines
                            for _ in 0..lines_to_add {
                                self.discrete_strip.lines.push(DiscreteLine::new(self.discrete_strip.get_next_id()));
                            }
                            log::info!(
                                "[SDR] no placement found, extending strip width by 10% to {:.3}",
                                new_width
                            );
                            sp_problem.modify_strip_in_back(new_width);
                        }
                    },
                }
            }
        }
        match &mut self.problem {
            Problem::BP(_) => {}
            Problem::SP(sp_problem) => {
                // sp_problem.strip_width();
                // log::info!(
                //     "[LBF] fitted strip width to {:.3}",
                //     sp_problem.strip_width()
                // );
            }
        }

        let elapsed_time = now.elapsed();
        println!("Runningtook {:?} ", elapsed_time);
        let solution= self.problem.create_solution(None);


        solution
    }

    pub fn discrete_placement(
        problem: &Problem,
        item: &Item,
        resolution: &f32,
        discrete_strip: &mut DiscreteStrip,
    ) -> Option<PlacingOption>{
        let discrete_segments = item.discretize_shape(*resolution, 0.0);
        match problem {
            Problem::BP(_) => {}
            Problem::SP(sp_problem) => {
                if let Some(transform) = discrete_strip.try_fit_segments(
                    discrete_segments, 
                    sp_problem.strip_height(), 
                    *resolution
                ) {
                    let translation_1 = transform.decompose().translation; 
                    let translation_2 = item.move_to_first_quadrant(0.0).decompose().translation; 
                    let added_translation = (
                        translation_1.0.into_inner() + translation_2.0.into_inner(), 
                        translation_1.1.into_inner() + translation_2.1.into_inner(), 
                    );
                    let new_transformation = DTransformation::new(
                        0.0,            // Keep the original rotation
                        added_translation,              // Use the added translation
                    );
                    return Some(PlacingOption {
                        layout_idx: STRIP_LAYOUT_IDX,  // For strip packing, use the constant
                        item_id: item.id,
                        d_transf: new_transformation,
                    });
                }
            }
        }
        
        None
    }

}