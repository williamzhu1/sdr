use std::f32::EPSILON;

use jagua_rs::geometry::transformation::Transformation;
use ordered_float::Float;


#[derive(Debug)]
pub struct Interval{
    pub start: f32,
    pub end: f32,
    pub orientation: Option<bool>,
}
#[derive(Debug)]
pub struct DiscreteLine{
    pub id: usize,
    pub occupied: Vec<Interval>
}

#[derive(Debug)]
pub struct DiscreteStrip{
    pub lines: Vec<DiscreteLine>
}
impl Interval{
    pub fn new(start: f32, end: f32) -> Self {
        Interval {
            start,
            end,
            orientation: None,
        }
    }
    pub fn new_with_orientation(start: f32, end: f32, orientation: bool) -> Self {
        Interval {
            start,
            end,
            orientation: Some(orientation),
        }
    }
    // Check if two intervals overlap
    pub fn overlaps(&self, other: &Interval) -> bool {
        match (self.orientation, other.orientation) {
            (Some(self_orient), Some(other_orient)) => {
                // If orientations are different, they can coexist (no overlap)
                if self_orient != other_orient {
                    return false;
                }
                // Same orientation continues to spatial check
            },
            // If either doesn't have orientation info, proceed to spatial check
            _ => {}
        }
        return self.start.max(other.start) <= self.end.min(other.end)
    }
     // Compute required shift to eliminate overlap
    pub fn shift_required(&self, other: &Interval) -> f32 {
        self.end - other.start + EPSILON
    }
    /// Create a new interval shifted by an offset
    pub fn shifted(&self, offset: f32) -> Interval {
        Interval::new(self.start + offset, self.end + offset)
    }
}

impl DiscreteLine {
    pub fn new(id:usize) -> Self {
        DiscreteLine{
        id,
        occupied: Vec::new()
        }
    }
    pub fn is_empty(&self) -> bool {
        self.occupied.is_empty()
    }
    pub fn find_first_available_position(&self, height: f32, incoming_line: &DiscreteLine) -> Option<f32> {
        // Check if the line is valid
        if self.occupied.is_empty() {
            return Some(0.0);
        }
        
        if &self.occupied.first().unwrap().start >= &incoming_line.occupied.last().unwrap().start{
            return Some(0.0);
        }
        let mut total_shift = 0.0;

        for segment in &self.occupied {
            // Only check segments that overlap with the movable segment
            if segment.overlaps(incoming_line.occupied.last().unwrap()) {
                let shift = segment.shift_required(incoming_line.occupied.last().unwrap());
                total_shift = total_shift.max(shift);
                
                // Check if this shift would exceed height
                if total_shift + incoming_line.occupied.last().unwrap().end > height {
                    return None;
                }
            }
        }
        Some(total_shift)
    }

    pub fn check_at(&self, height: f32, incoming_line: &DiscreteLine, offshoot: f32) -> Option<f32>{
        // Shift incoming line's intervals
        let last_interval = match incoming_line.occupied.last() {
            Some(interval) => interval.shifted(offshoot),
            None => return Some(0.0), // No segments in incoming line
        };
        // Check height constraint
        if last_interval.end > height {
            return None;
        }
        // overlap check
        for current_interval in &self.occupied {
            if current_interval.overlaps(&last_interval) {
                let shift_needed = current_interval.shift_required(&last_interval);
                log::info!("please shift {:?}, {:?}", current_interval, &last_interval);
                if shift_needed > EPSILON {
                    return Some(shift_needed);
                }                
            }
        }
        Some(0.0)
    }
}

impl DiscreteStrip {
    pub fn get_next_id(&self) -> usize {
        self.lines.last().map_or(0, |last_line| last_line.id + 1)
    }
    pub fn try_fit_segments(&mut self, segments: Vec<DiscreteLine>, height: f32, resolution: f32) -> Option<Transformation>{
        log::info!("Height: {:?}, Fitting segments {:?}", height, segments);
        let first = &segments[0];
        let mut final_offshot = 0.0;
        
            'outer: for line in &mut self.lines.iter() {
                if let Some(offshoot) = line.find_first_available_position(height, first) {
                    let mut stack: Vec<f32> = Vec::new();
                    stack.push(0.0);
                    let i0: usize = line.id;
                    'stacked: while !stack.is_empty() {
                        let mut i: usize = line.id;
                        let mut current_id = 0;
                        log::info!("i0: {:?}, offset", i0);
                        for segment in segments.iter() {
                            if segment.id != current_id {
                                i += 1;
                                current_id = segment.id;
                            }
                            if i >= self.lines.len() {
                                break 'outer; // Or handle this case differently
                            }
                            match self.lines[i].check_at(height, segment, offshoot + stack[0]) {
                                // If check_at returns None, the segment cannot fit, so continue the outer loop.
                                None => {
                                    log::info!("Segment {:?} does not fit at offset {:?}", segment, offshoot);
                                    stack.pop();
                                    continue 'outer;
                                },
                                
                                // If check_at returns 0.0, the segment fits without shift, so continue to the next segment.
                                Some(0.0) => {
                                    log::info!("Segment {:?} fits without shift", segment);                             
                                },

                                // If check_at returns a non-zero shift value, we need to apply the shift.
                                Some(shift) => {
                                    if shift > EPSILON {
                                        log::info!("Segment {:?} requires a shift of {:?}", segment, shift);
                                        stack.insert(0, shift + stack[0]);
                                        stack.pop();
                                        continue 'stacked;
                                    }
                                    log::info!("Segment {:?} EPSILON", segment);
                                }
                            }
                        }
                        final_offshot = offshoot + stack[0];
                        stack.pop();                       
                    }     
                    log::info!("Fitting at {:?}, the segments {:?}, with offshot {:?}", i0, segments, final_offshot);
                    self.add_segments(segments, final_offshot, i0);
                    log::info!("Fitting success {:?}", self);
                    return Some(Transformation::from_translation((i0 as f32 * resolution, height - final_offshot)));
                }
            }
        log::info!("Fitting failed");
        None
    }

    pub fn add_segments(&mut self, segments: Vec<DiscreteLine>, offshoot: f32, firstline: usize) {
        for (j, segment) in segments.iter().enumerate() {
            if let Some(line) = self.lines.get_mut(firstline + j) {
                // Process each interval from the segment
                for interval in segment.occupied.iter() {
                    // Create modified interval with offset
                    let new_interval = Interval {
                        start: interval.start + offshoot,
                        end: interval.end + offshoot,
                        orientation: interval.orientation,
                    };
                    
                    // Check if we need to split any existing intervals
                    let mut split_operations = Vec::new();
                    
                    for i in 0..line.occupied.len() {
                        let existing = &line.occupied[i];
                        
                        // Check for overlap
                        if existing.start <= new_interval.end && existing.end >= new_interval.start {
                            // If orientations are different and both are Some, we need to split
                            if let (Some(exist_orient), Some(new_orient)) = (existing.orientation, new_interval.orientation) {
                                if exist_orient != new_orient {
                                    // Record split operation for later execution
                                    split_operations.push((i, new_interval.start.max(existing.start), new_interval.end.min(existing.end)));
                                }
                            }
                        }
                    }
                    
                    // Execute splits (in reverse to keep indices valid)
                    for (i, split_start, split_end) in split_operations.iter().rev() {
                        let existing = line.occupied.remove(*i);
                        
                        // Add parts before split
                        if existing.start < *split_start {
                            line.occupied.push(Interval {
                                start: existing.start,
                                end: *split_start,
                                orientation: existing.orientation,
                            });
                        }
                        
                        // Add split part with new orientation
                        line.occupied.push(Interval {
                            start: *split_start,
                            end: *split_end,
                            orientation: new_interval.orientation,
                        });
                        
                        // Add parts after split
                        if existing.end > *split_end {
                            line.occupied.push(Interval {
                                start: *split_end,
                                end: existing.end,
                                orientation: existing.orientation,
                            });
                        }
                    }
                    
                    // If no splits were made, try to extend an existing interval or add as new
                    if split_operations.is_empty() {
                        let mut merged = false;
                        
                        // Try extending existing intervals
                        for i in 0..line.occupied.len() {
                            let existing = &mut line.occupied[i];
                            
                            // Check if they can be merged (same orientation and touching)
                            if existing.orientation == new_interval.orientation {
                                if existing.end == new_interval.start {
                                    // Existing ends where new starts - extend end
                                    existing.end = new_interval.end;
                                    merged = true;
                                    break;
                                } else if existing.start == new_interval.end {
                                    // Existing starts where new ends - extend start
                                    existing.start = new_interval.start;
                                    merged = true;
                                    break;
                                }
                            }
                        }
                        
                        // If not merged with any existing interval, add as new
                        if !merged {
                            line.occupied.push(new_interval);
                        }
                    }
                }
                
                // Sort by start position
                line.occupied.sort_by(|a, b| a.start.partial_cmp(&b.start).unwrap());
            }
        }
    }  
}

