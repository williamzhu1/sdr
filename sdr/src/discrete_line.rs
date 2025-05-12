// use std::arch::x86_64::__m512;
// // use std::simd::{f32x16, mask32x16, Simd};
use rayon::prelude::*;

use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use jagua_rs::{entities::item, fsize, geometry::transformation::Transformation};
use ordered_float::Float;
pub const EPSILON:fsize = 0.000001;

static OVERLAPS_COUNTER: AtomicUsize = AtomicUsize::new(0);

// Function to get the current count
pub fn get_overlaps_count() -> usize {
    OVERLAPS_COUNTER.load(Ordering::Relaxed)
}

#[derive(Debug, Clone, Copy)]
pub struct Interval{
    pub start: fsize,
    pub end: fsize,
    pub orientation: Option<bool>,
    total_space: fsize,
}

#[derive(Debug, Clone)]
pub struct DiscreteLine{
    pub occupied: Vec<Interval>,
}

#[derive(Debug, Clone)]
pub struct DiscreteObject{
    pub id: usize,
    pub lines: Vec<DiscreteLine>,
    pub position: (usize, fsize),
}

#[derive(Debug, Clone)]
pub struct DiscreteStrip{
    pub lines: Vec<DiscreteLine>
}

impl Interval{
    
    pub fn new(start: fsize, end: fsize) -> Self {
        Interval {
            start,
            end,
            orientation: None,
            total_space: end - start,
        }
    }

    pub fn new_with_orientation(start: fsize, end: fsize, orientation: bool) -> Self {
        Interval {
            start,
            end,
            orientation: Some(orientation),
            total_space: end - start,
        }
    }

    #[inline(always)]
    pub fn overlaps(&self, other: &Interval) -> fsize {
        OVERLAPS_COUNTER.fetch_add(1, Ordering::Relaxed);
        // Optimize spatial overlap check
        let spatial_overlap = if self.end == self.start || other.start == other.end {
            (self.end.min(other.end) - self.start.max(other.start)) > -EPSILON
        } else {
            (self.end.min(other.end) - self.start.max(other.start)) > EPSILON
        };
        
        if !spatial_overlap {
            return 0.0;
        }
        
        // If there is spatial overlap, check orientations
        match (self.orientation, other.orientation) {
            (Some(self_orient), Some(other_orient)) => {
                if self_orient != other_orient {
                    return 0.0;
                }
            }
            _ => {}
        }
        self.end - other.start 
    }

    #[inline(always)]
    pub fn shifted(&self, offset: &fsize) -> Interval {
        Interval {
            start: self.start + offset,
            end: self.end + offset,
            orientation: self.orientation,
            total_space: self.total_space,
        }
    }
}

impl DiscreteLine {
    
    pub fn new() -> Self {
        DiscreteLine{
        occupied: Vec::new()
        }
    }
    
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.occupied.is_empty()
    }

    #[inline(always)]
    pub fn add_interval(&mut self, interval: Interval) {
        // Find insertion point to maintain sorted order by start position
        let pos = match self.occupied.binary_search_by(|x| {
            match x.start.partial_cmp(&interval.start) {
                Some(std::cmp::Ordering::Equal) => {
                    // If starts are equal, compare by end 
                    x.end.partial_cmp(&interval.end).unwrap_or(std::cmp::Ordering::Equal)
                },
                Some(order) => order,
                None => std::cmp::Ordering::Equal
            }
        }) {
            Ok(i) => i,
            Err(i) => i,
        };
        
        // Insert at the correct position
        self.occupied.insert(pos, interval);
    }

    #[inline(always)]
    pub fn space(&self) -> fsize {
        self.occupied.iter().map(|interval| interval.total_space).sum()
    }

    #[inline(always)]
    pub fn free_space(&self, height: fsize) -> fsize {
        height - self.space()
    }

    #[inline(always)]
    pub fn check_at(&self, height: &fsize, incoming_line: &DiscreteLine, offshoot: &fsize) -> Option<fsize>{

        let mut max_shift_needed = 0.0;

        // Sort incoming intervals by start position
        let mut search_idx = 0; // Start at the beginning
        
        for incoming_interval in &incoming_line.occupied {
            let shifted_interval = incoming_interval.shifted(offshoot);
            
            // Line height check
            if shifted_interval.end > *height + EPSILON{
                return None;
            }
            
            // Skip intervals until we find potential overlaps
            while search_idx < self.occupied.len() && self.occupied[search_idx].end <= shifted_interval.start {
                search_idx += 1;
            }
            
            // Check from this position forward
            let mut i = search_idx;
            while i < self.occupied.len() {
                let current = &self.occupied[i];
                if current.start > shifted_interval.end {
                    break;
                }
                
                let shift = current.overlaps(&shifted_interval);
                if shift > EPSILON {
                    max_shift_needed = max_shift_needed.max(shift);
                }
                i += 1;
            }
        }  
        
        // Final height check with calculated shift
        for interval in &incoming_line.occupied {
            if interval.end + offshoot + max_shift_needed > *height + EPSILON{
                return None;
            }
        }
        
        Some(max_shift_needed)
    }

    
}

impl DiscreteStrip {

    pub fn get_next_id(&self) -> usize {
        self.lines.len() + 1
    }


    pub fn try_fit_segments(&mut self, polygon_sets: &Vec<&Vec<DiscreteLine>>, height: &fsize, resolution: &fsize, amount: &isize, item_id: &usize) -> Vec<(Transformation, usize)> {
        if polygon_sets.is_empty() {
            return Vec::new();
        }
        
        self.check_polygon(polygon_sets, height, resolution, amount, item_id)
    }

    pub fn check_polygon(&mut self, polygon_sets: &Vec<&Vec<DiscreteLine>>, height: &fsize, resolution: &fsize, amount: &isize, item_id: &usize) -> Vec<(Transformation, usize)>{
        if !is_x86_feature_detected!("avx2"){       
            unsafe {return self.check_polygon_avx2(polygon_sets, height, resolution, amount, item_id);}
        } else{   
            return self.check_polygon_scalar(polygon_sets, height, resolution, amount, item_id);
        }
    }

    #[target_feature(enable = "avx2")]
    unsafe fn check_polygon_avx2(&mut self, polygon_sets: &Vec<&Vec<DiscreteLine>>, height: &fsize, resolution: &fsize, amount: &isize,  item_id: &usize) -> Vec<(Transformation, usize)>{
        return self.check_polygon_scalar(polygon_sets, height, resolution, amount, item_id);
    }

    #[inline(always)]
    fn check_polygon_scalar(&mut self, polygon_sets: &Vec<&Vec<DiscreteLine>>, height: &fsize, resolution: &fsize, amount: &isize, item_id: &usize) -> Vec<(Transformation, usize)>{
        let mut results: Vec<(Transformation, usize)> = Vec::with_capacity(*amount as usize);
        let start_time = Instant::now();
    
        let mut placed = *amount;
        let mut start_x = 0;
        let mut start_offshoot = 0.0;

        while placed > 0 {
            let placing_time = Instant::now();
            let mut best_placement = None;
            // Process each set of segments
            for (i,polygon) in polygon_sets.iter().enumerate(){
                let placement = self.find_best_position(polygon, height, start_x, start_offshoot, &i);
                match best_placement {
                    None => {
                        best_placement = placement;
                    },
                    Some((best_i0, best_offshoot, best_set_index)) => {
                        let tentative = placement.unwrap();
                        
                        // Compare based on line index, then offshoot, then empty space
                        if tentative.0 < best_i0 || 
                        (tentative.0 == best_i0 && tentative.1 < best_offshoot) ||
                        (tentative.0 == best_i0 && tentative.1 == best_offshoot && polygon_sets[tentative.2].first().unwrap().space() < polygon_sets[best_set_index].first().unwrap().space()) {
                            best_placement = placement;
                        }
                    }
                }
            }
            if let Some((best_i0, best_offshoot, best_set_index)) = best_placement {
                // Apply the selected placement
                let selected_segments = polygon_sets[best_set_index];
                self.add_segments(selected_segments, best_offshoot, best_i0);
                start_x = best_i0;
                start_offshoot = best_offshoot;
                placed = placed - 1;
                //println!("placing item {} {} took {:?} ",item_id ,placed , placing_time.elapsed());
                results.push((Transformation::from_translation((best_i0 as fsize * resolution, height - best_offshoot)), best_set_index));
            } else{
                println!("ERROR");
                break;
            }
        }
        //println!("checking took {:?} ", start_time.elapsed());
        results
    }

    #[inline(always)]
    fn find_best_position(&self, polygon: &Vec<DiscreteLine>, height: &fsize, start_x: usize, start_offshoot: fsize, set_index: &usize) ->  Option<(usize, fsize, usize)>{
        // Skip empty segment sets
        let mut initial_offshoot = start_offshoot;
    
        let first: &DiscreteLine = &polygon[0];
        
        // Try each available line starting from start_x
        for line_idx in start_x..self.lines.len() {
            let line = &self.lines[line_idx];
            
            // Check if we can fit the first segment at this line
            if let Some(offshoot_shift) = line.check_at(height, first, &initial_offshoot) {
                let offshoot = initial_offshoot + offshoot_shift;
                
                // Now verify if all segments fit
                if let Some(final_offshoot) = self.verify_all_segments(polygon, height, line_idx, offshoot) {
                    return Some((line_idx, final_offshoot, *set_index));
                }
            }
            
            // Reset offshoot for next line
            initial_offshoot = 0.0;
        }
        None
    }
    
    #[inline(always)]
    fn verify_all_segments(&self, polygon: &Vec<DiscreteLine>, height: &fsize, i0: usize, offshoot: fsize) -> Option<f32> {
        let mut current_offshoot = offshoot;
        
        'outer: loop {    
            let mut all_fit = true;
            for (i, segment) in polygon.iter().enumerate() {                
                // Check if segment fits at current line and offshoot
                match self.lines[i0 + i].check_at(height, segment, &current_offshoot) {
                    None => return None, // Segment doesn't fit
                    Some(shift) => {
                        if shift > EPSILON {
                            // Need to shift and retry all segments
                            current_offshoot += shift;
                            all_fit = false;
                            break;
                        }
                    }
                }
            }
            if all_fit {
                return Some(current_offshoot);
            }

            if current_offshoot > *height {
                return None;
            }
        }
    }

    #[inline(always)]
    pub fn add_segments(&mut self, segments: &Vec<DiscreteLine>, offshoot: fsize, firstline: usize) {
        for (j, segment) in segments.iter().enumerate() {
            if let Some(line) = self.lines.get_mut(firstline + j) {
                // Process each interval from the segment
                for interval in segment.occupied.iter() {
                    // Create modified interval with offset
                    let new_interval = Interval {
                        start: interval.start + offshoot,
                        end: interval.end + offshoot,
                        orientation: interval.orientation,
                        total_space: interval.total_space,
                    };
    
                    line.add_interval(new_interval);
                
                }
            }
        }
    }  

    pub fn trim_after_last_occupied(&mut self) {
        // Find the index of the last line with non-empty occupied list
        let last_occupied_index = self.lines.iter()
            .enumerate()
            .rfind(|(_, line)| !line.occupied.is_empty())
            .map(|(index, _)| index);
        
        if let Some(index) = last_occupied_index {
            // Keep only up to and including the last occupied line
            self.lines.truncate(index + 1);
        }
        // If no occupied lines are found, keep all lines as they are
    }



    // unsafe fn check_simd(&self, polygon: &Vec<DiscreteLine>, height: &fsize, start_x: usize, start_offshoot: fsize, segment_heights: &fsize, set_index: &usize) ->  Option<(usize, fsize, usize)>{
        
    //     type SimdType = __m512;
    //     todo!()
    // }
}