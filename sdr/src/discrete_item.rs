use std::time::Instant;
use jagua_rs::fsize;
use jagua_rs::geometry::primitives::edge::{self, Edge};
use jagua_rs::geometry::primitives::simple_polygon::SimplePolygon;
use jagua_rs::{entities::item::Item, geometry::{geo_traits::Transformable, primitives::point::Point, transformation::Transformation}};
use ordered_float::NotNan;

use crate::discrete_line::{DiscreteLine, Interval};

// Define a trait to be implemented for Item
pub trait Discretizable {
    fn cross_product(&self, o: &Point, a: &Point, b: &Point) -> fsize;
    fn discretize_shape(&self, resolution: fsize, rotation: fsize) -> Vec<DiscreteLine>;
    fn sweep_line_algorithm(&self, resolution: fsize, first_quadrant: &SimplePolygon) -> Vec<DiscreteLine>;
    fn convex_vertex(&self) -> Vec<usize>;
    fn move_to_first_quadrant(&self, rotation: fsize) -> Transformation;
    fn process_edge_extensions(&self, discretized_lines: &mut Vec<DiscreteLine>, edge: &Edge, 
        vertex: &Point, left_line_idx: usize, right_line_idx: usize,
        left_line_x: fsize, right_line_x: fsize, resolution: fsize);
    fn extend_convex_vertex(&self, discretized_lines: &mut Vec<DiscreteLine>, vertex: &Point, v_index: usize, first_quadrant: &SimplePolygon, resolution: fsize);
    fn add_extension_interval(&self, line_idx: usize, interval: Interval, discretized_lines: &mut Vec<DiscreteLine>);
    fn apply_extension(&self, discretized_lines: &mut Vec<DiscreteLine>, first_quadrant: &SimplePolygon, resolution: fsize);
}

// Implement the trait for Item
impl Discretizable for Item {
    fn sweep_line_algorithm(&self, resolution: fsize, first_quadrant: &SimplePolygon) -> Vec<DiscreteLine> {
        const EPSILON: f32 = 0.000001;
        let rect = &first_quadrant.bbox;
        let mut results = Vec::new();
        let mut x_line = rect.x_min;
        let mut line_counter: usize = 0;
        
        while x_line <= rect.x_max + EPSILON + resolution{
            let mut last_intersection: Option<fsize> = None;
            let mut merged_intersections: Vec<Interval> = Vec::new();
            let mut edges: Vec<Edge> = Vec::new();
            let mut vertical_edges: Vec<Edge> = Vec::new();

            for edge in first_quadrant.edge_iter() {
                let (x_low, x_high) = if edge.start.0 < edge.end.0 {
                    (edge.start.0, edge.end.0)
                } else {
                    (edge.end.0, edge.start.0)
                };

                if x_line >= rect.x_max + EPSILON && x_high >= rect.x_max - EPSILON{
                    println!("LAST");
                    if (edge.start.0 - edge.end.0).abs() < EPSILON {
                        merged_intersections.push(Interval::new_with_orientation(edge.end.1.abs(), edge.start.1.abs(), false));
                        continue;
                    }
                }

                if x_line < x_low - EPSILON || x_line > x_high + EPSILON  {
                    continue;
                }

                // Handle direct vertex intersections
                if (edge.start.0 - edge.end.0).abs() < EPSILON {
                    vertical_edges.push(edge);
                    continue;
                }
                match edges.last(){
                    Some(last_edge) =>{
                        
                        // if (last_edge.end.0 - edge.start.0).abs() < EPSILON && (last_edge.end.1 - edge.start.1).abs() < EPSILON {
                        //     continue;
                        // }
                        edges.push(edge);    
                    },
                    None =>{

                        edges.push(edge);    
                    }
                }
                
            }
            
            if !vertical_edges.is_empty(){
                for vertical in vertical_edges{
                    if vertical.start.1 < vertical.end.1 {
                        merged_intersections.push(Interval::new_with_orientation(vertical.end.1.abs(), vertical.start.1.abs(), false));
                    }
                    else{
                        merged_intersections.push(Interval::new_with_orientation(vertical.start.1.abs(), vertical.end.1.abs(), true));
                    }
                    let new_edges: Vec<Edge> = edges.iter().filter(|edge1| {
                        // Keep an edge only if it doesn't share any point with the vertical edge
                        !approx_eq(&edge1.start, &vertical.start) && 
                        !approx_eq(&edge1.start, &vertical.end) && 
                        !approx_eq(&edge1.end, &vertical.start) && 
                        !approx_eq(&edge1.end, &vertical.end)
                    }).cloned().collect();
                    if !new_edges.is_empty(){
                        for e in new_edges{
                            let t = (x_line - e.start.0) / (e.end.0 - e.start.0);
                            let y_intersect = e.start.1 + t * (e.end.1 - e.start.1);
                            if t >= 0.0 && t <= 1.0{
                                if y_intersect.abs() > merged_intersections.last().unwrap().end {
                                    merged_intersections.push(Interval::new(merged_intersections.last().unwrap().end, y_intersect.abs()));
                                }
                                else{
                                    merged_intersections.push(Interval::new(y_intersect.abs(), merged_intersections.last().unwrap().start));
                                }
                            } 
                        }
                    }
                }
            }
            else{
                let mut orientation: bool = true;
                for (i,edge) in edges.iter().enumerate(){
                    let t = (x_line - edge.start.0) / (edge.end.0 - edge.start.0);
                    let y_intersect = edge.start.1 + t * (edge.end.1 - edge.start.1);
                    if t >= 0.0 && t <= 1.0{
                        if edge.end.1 > edge.start.1{
                            orientation = false;
                        }
                        else{
                            orientation = true;
                        }
                        if let Some(last_y_val) = last_intersection {
                            if last_y_val != y_intersect {
                                if last_y_val < y_intersect{
                                    merged_intersections.push(Interval::new(y_intersect.abs(), last_y_val.abs()));
                                }
                                else {
                                    merged_intersections.push(Interval::new(last_y_val.abs(), y_intersect.abs()))
                                }
                                last_intersection = None; // Reset for next pair
                            }
                        } else {
                            last_intersection = Some(y_intersect);
                        }
                    }
                }
                if let Some(last_y_val) = last_intersection{
                    merged_intersections.push(Interval::new_with_orientation(last_y_val.abs(), last_y_val.abs(), orientation));
                }
            }
            // println!("Convex {:?}", convex);
            // if x_line > rect.x_max{
            //     for vertex in convex.iter(){
            //         let current = first_quadrant.get_point(*vertex);
            //         if current.0 >= prev_line && current.0 <= next_line{
            //             println!("Extension needed {:?}", current);
            //         }
            //     }

            // }else{
            //     for(i, vertex) in convex.iter().enumerate(){
            //         let current = first_quadrant.get_point(*vertex);
            //         if current.0 >= prev_line && current.0 <= next_line{
            //             println!("Extension needed {:?}", current);
            //         }
            //     }
            // }
            
            // println!("Intersections for line {:?} :{:?}",x_line , merged_intersections);


            if !merged_intersections.is_empty() {
                for intersection in merged_intersections.into_iter(){
                    results.push(DiscreteLine{id: line_counter, occupied: vec![intersection]});
                }
            }           
            x_line += resolution;
            line_counter = line_counter + 1;
        }
        
        results
    }

    fn cross_product(&self, o: &Point, a: &Point, b: &Point) -> fsize {
        let ax = a.0 - o.0;
        let ay = a.1 - o.1;
        let bx = b.0 - o.0;
        let by = b.1 - o.1;
    
        ax * by - ay * bx
    }
    fn convex_vertex(&self) -> Vec<usize> {
        let mut vertices = Vec::new();
        let mut i = 0;
        for vertex in &self.shape.points{
            let prev = if i == 0 { &self.shape.number_of_points() - 1 } else { i - 1 };
            let next = if i == &self.shape.number_of_points() - 1 { 0 } else { i + 1 };
            let cross = self.cross_product(&self.shape.get_point(prev), vertex, &self.shape.get_point(next));
            if cross > 0.0{
                vertices.push(i);
            }
            i += 1;
        }
        // log::info!("Convex Vertices {:?}", vertices);
        vertices
    }
    fn move_to_first_quadrant(&self, rotation: fsize) -> Transformation {
        let rotate = Transformation::from_rotation(rotation);
        if rotation != 0.0 {
            let rotated_shape = self.shape.transform_clone(&rotate);
            let min_x = rotated_shape.points.iter().map(|p| p.0).fold(fsize::MAX, fsize::min);
            let min_y = rotated_shape.points.iter().map(|p| p.1).fold(fsize::MAX, fsize::min);
            let translation = Transformation::from_translation((-min_x, -min_y - self.shape.bbox.y_max + self.shape.bbox.y_min));
            return rotate.transform(&translation);
        }
        else{
            let min_x = self.shape.points.iter().map(|p| p.0).fold(fsize::MAX, fsize::min);
            let min_y = self.shape.points.iter().map(|p| p.1).fold(fsize::MAX, fsize::min);
            let translation = Transformation::from_translation((-min_x, -min_y - self.shape.bbox.y_max + self.shape.bbox.y_min));
            return translation;
        }
    }
    fn discretize_shape(&self, resolution: fsize, rotation: fsize) -> Vec<DiscreteLine> {
        // let discretizationtime = Instant::now();
        let first_quadrant = self.shape.transform_clone(&self.move_to_first_quadrant(rotation));
        let mut results = self.sweep_line_algorithm(resolution, &first_quadrant); 
        self.apply_extension(&mut results, &first_quadrant, resolution);
        // log::info!("Time to discretize item processing time: {:?}, rotation: {:?}", discretizationtime.elapsed(), rotation);

        results
    }
    fn extend_convex_vertex(&self, discretized_lines: &mut Vec<DiscreteLine>, vertex: &Point, v_index: usize, 
        first_quadrant: &SimplePolygon, resolution: fsize) {
        const EPSILON: fsize = 0.000001;

        // Skip vertices that are exactly on resolution lines (they're already handled in sweep line)
        let x_pos = vertex.0;
        let x_mod = x_pos % resolution;
        if x_mod < EPSILON || resolution - x_mod < EPSILON {
            return;
        }

        // Find which resolution lines this vertex lies between
        let left_line_idx = (x_pos / resolution).floor() as usize;
        let right_line_idx = left_line_idx + 1;

        if left_line_idx >= discretized_lines.len() || right_line_idx >= discretized_lines.len() {
            return; // Out of bounds
        }

        let left_line_x = left_line_idx as fsize * resolution;
        let right_line_x = right_line_idx as fsize * resolution;

        // Get adjacent edges to this vertex
        let prev_index = if v_index == 0 { first_quadrant.number_of_points() - 1 } else { v_index - 1 };
        let next_index = if v_index == first_quadrant.number_of_points() - 1 { 0 } else { v_index + 1 };

        let prev_point = first_quadrant.get_point(prev_index);
        let next_point = first_quadrant.get_point(next_index);

        let edge1 = Edge::new(prev_point, *vertex);
        let edge2 = Edge::new(*vertex, next_point);

        // Case 1: If edge intersects a neighboring resolution line
        // For edge1
        self.process_edge_extensions(discretized_lines, &edge1, vertex, left_line_idx, right_line_idx, 
                        left_line_x, right_line_x, resolution);

        // For edge2
        self.process_edge_extensions(discretized_lines, &edge2, vertex, left_line_idx, right_line_idx, 
                        left_line_x, right_line_x, resolution);
    }

    fn process_edge_extensions(&self, discretized_lines: &mut Vec<DiscreteLine>, edge: &Edge, 
        vertex: &Point, left_line_idx: usize, right_line_idx: usize,
        left_line_x: fsize, right_line_x: fsize, resolution: fsize) {
        const EPSILON: fsize = 0.000001;

        // Check if edge intersects left resolution line
        if (edge.start.0 < left_line_x + EPSILON && edge.end.0 > left_line_x - EPSILON) || 
        (edge.end.0 < left_line_x + EPSILON && edge.start.0 > left_line_x - EPSILON) {
            // Calculate intersection point
            let t = (left_line_x - edge.start.0) / (edge.end.0 - edge.start.0);
            let y_intersect = edge.start.1 + t * (edge.end.1 - edge.start.1);

            if t >= 0.0 && t <= 1.0 {
                // Add extension interval on left line: (y_A, y_v, R)
                let left_interval = Interval::new_with_orientation(
                y_intersect.abs().min(vertex.1.abs()),
                y_intersect.abs().max(vertex.1.abs()),
                false // Right position
                );
                self.add_extension_interval(left_line_idx, left_interval, discretized_lines);

                // Add point extension on right line: (y_v, y_v, L)
                let right_interval = Interval::new_with_orientation(
                vertex.1.abs(),
                vertex.1.abs(),
                true // Left position
                );
                self.add_extension_interval(right_line_idx, right_interval, discretized_lines);
            }
        }
        // Check if edge intersects right resolution line
        else if (edge.start.0 < right_line_x + EPSILON && edge.end.0 > right_line_x - EPSILON) || 
        (edge.end.0 < right_line_x + EPSILON && edge.start.0 > right_line_x - EPSILON) {
            // Calculate intersection point
            let t = (right_line_x - edge.start.0) / (edge.end.0 - edge.start.0);
            let y_intersect = edge.start.1 + t * (edge.end.1 - edge.start.1);

            if t >= 0.0 && t <= 1.0 {
                // Add extension interval on right line: (y_A, y_v, L)
                let right_interval = Interval::new_with_orientation(
                y_intersect.abs().min(vertex.1.abs()),
                y_intersect.abs().max(vertex.1.abs()),
                true // Left position
                );
                self.add_extension_interval(right_line_idx, right_interval, discretized_lines);

                // Add point extension on left line: (y_v, y_v, R)
                let left_interval = Interval::new_with_orientation(
                vertex.1.abs(),
                vertex.1.abs(),
                false // Right position
                );
                self.add_extension_interval(left_line_idx, left_interval, discretized_lines);
            }
        }
        // Case 2: Edge doesn't intersect either resolution line (whole edge between lines)
        else if edge.start.0 > left_line_x - EPSILON && edge.end.0 < right_line_x + EPSILON {
            // Project the edge orthogonally onto both resolution lines
            let y_start = edge.start.1.abs();
            let y_end = edge.end.1.abs();

            // Add extension intervals on both lines
            let left_interval = Interval::new_with_orientation(
            y_start.min(y_end),
            y_start.max(y_end),
            false // Right position
            );
            self.add_extension_interval(left_line_idx, left_interval, discretized_lines);

            let right_interval = Interval::new_with_orientation(
            y_start.min(y_end),
            y_start.max(y_end),
            true // Left position
            );
            self.add_extension_interval(right_line_idx, right_interval, discretized_lines);
        }
    }

    fn add_extension_interval(&self, line_idx: usize, interval: Interval, discretized_lines: &mut Vec<DiscreteLine>) {
        if line_idx >= discretized_lines.len() {
            return; // Out of bounds
        }
        let existing_intervals = &discretized_lines[line_idx].occupied;

        // If fully covered by any existing, drop it entirely
        for ex in existing_intervals.iter() {
            if interval.start >= ex.start && interval.end <= ex.end {
                return;
            }
        }

        // We'll keep a list of "remaining pieces" of our new interval
        // starting with the full thing
        let mut remaining = vec![(interval.start, interval.end)];

        // For each existing, carve out overlapping bits
        for ex in existing_intervals.iter() {
            let mut next_remaining = Vec::new();
            for &(rs, re) in &remaining {
                // No overlap?
                if re <= ex.start || rs >= ex.end {
                    // keep the whole piece
                    next_remaining.push((rs, re));
                } else {
                    // overlap exists; carve out the intersection
                    if rs < ex.start {
                        // left slice remains
                        next_remaining.push((rs, ex.start));
                    }
                    if re > ex.end {
                        // right slice remains
                        next_remaining.push((ex.end, re));
                    }
                    // the middle bit [max(rs,ex.start), min(re,ex.end)] is dropped
                }
            }
            remaining = next_remaining;
            if remaining.is_empty() {
                // nothing left to add
                return;
            }
        }

        // Whatever is left in `remaining` is non-overlapping; add them
        for (rs, re) in remaining {
            let pos = interval.orientation.unwrap(); 
            let new_int = Interval::new_with_orientation(rs, re, pos);
            discretized_lines[line_idx].occupied.push(new_int);
        }

        // Finally, re‐sort by start
        discretized_lines[line_idx].occupied.sort_by(|a, b| {
            a.start.partial_cmp(&b.start).unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    fn apply_extension(&self, discretized_lines: &mut Vec<DiscreteLine>, first_quadrant: &SimplePolygon, resolution: fsize) {
        // Get convex vertices
        let convex_indices = self.convex_vertex();
        
        // Process each convex vertex for extension
        for &v_index in &convex_indices {
            let vertex = first_quadrant.get_point(v_index);
            self.extend_convex_vertex(discretized_lines, &vertex, v_index, first_quadrant, resolution);
        }
    }
    
}

fn approx_eq(p1: &Point, p2: &Point) -> bool {
    const EPSILON: fsize = 0.000001;
    (p1.0 - p2.0).abs() < EPSILON && (p1.1 - p2.1).abs() < EPSILON
}

