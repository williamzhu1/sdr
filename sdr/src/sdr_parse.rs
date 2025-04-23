use std::collections::HashMap;
use std::time::Instant;

use itertools::Itertools;
use jagua_rs::entities::instances::instance::Instance;
use jagua_rs::entities::instances::strip_packing::SPInstance;
use jagua_rs::entities::item::Item;
use jagua_rs::fsize;
use jagua_rs::geometry::d_transformation::DTransformation;
use jagua_rs::geometry::geo_enums::AllowedRotation;
use jagua_rs::geometry::geo_traits::{Shape, Transformable};
use jagua_rs::geometry::primitives::aa_rectangle::AARectangle;
use jagua_rs::geometry::primitives::point::Point;
use jagua_rs::geometry::primitives::simple_polygon::SimplePolygon;
use jagua_rs::geometry::transformation::Transformation;
use jagua_rs::io::json_instance::{JsonInstance, JsonItem, JsonShape, JsonSimplePoly};
use jagua_rs::util::config::CDEConfig;
use jagua_rs::util::polygon_simplification;
use jagua_rs::util::polygon_simplification::{PolySimplConfig, PolySimplMode};
use ordered_float::OrderedFloat;
use rayon::iter::IndexedParallelIterator;
use rayon::iter::ParallelIterator;
use rayon::prelude::*;
use std::sync::{Arc, Mutex};
use dashmap::DashMap;

use crate::discrete_item::Discretizable;
use crate::discrete_line::DiscreteLine;

#[derive(Debug)]
pub struct SdrParse {
    poly_simpl_config: PolySimplConfig,
    cde_config: CDEConfig,
    center_polygons: bool,
}

#[derive(Hash, Eq, PartialEq, Debug)]
pub struct ItemDiscrete{
    id: usize,
    rotation: OrderedFloat<fsize>,
}

impl ItemDiscrete{
    fn new(id: &usize, rotation: &fsize) -> ItemDiscrete{
        ItemDiscrete { id: *id, rotation: OrderedFloat(*rotation) }
    }
}

#[derive(Debug)]
pub struct SdrInstance{
    pub instance: Instance,
    pub items: HashMap<ItemDiscrete, Vec<DiscreteLine>>,
}

impl SdrInstance{
    fn new<'a>(instance: Instance, it: Vec<(Item,usize)>, resolution: fsize) -> SdrInstance {
        let mut items = DashMap::new();

        let start_time = Instant::now();
        // Iterate through each item in the instance
        it.par_iter().for_each(|item| {
            // let item_time = Instant::now();
            // Create an ItemDiscrete for this item
            match &item.0.allowed_rotation {
                AllowedRotation::Discrete(angles) => {
                    for rotation in angles {
                        let item_discrete = ItemDiscrete::new(&item.0.id, rotation);
                        let discrete_lines = item.0.discretize_shape(resolution, *rotation);
                        items.insert(item_discrete, discrete_lines);
                    }
                },
                _ => {
                    let item_discrete = ItemDiscrete::new(&item.0.id, &0.0);   
                    let discrete_lines = item.0.discretize_shape(resolution, 0.0); 
                    items.insert(item_discrete, discrete_lines);
                },
            }
        });
        log::info!("Total parallelized processing time: {:?}", start_time.elapsed());
        let items: HashMap<_, _> = items.into_iter().collect();
        
        SdrInstance {
            instance: instance.clone(), 
            items,
        }
    }
}

impl SdrParse{
    pub fn new(
        poly_simpl_config: PolySimplConfig,
        cde_config: CDEConfig,
        center_polygons: bool,
    ) -> SdrParse {
        SdrParse {
            poly_simpl_config,
            cde_config,
            center_polygons,
        }
    }

    pub fn parse(&self, json_instance: &JsonInstance, resolution: fsize) -> SdrInstance {
        let items: Vec<(Item, usize)> = json_instance
            .items
            .par_iter()
            .enumerate()
            .map(|(item_id, json_item)| self.parse_item(json_item, item_id))
            .collect();

        let instance: Instance = match (json_instance.bins.as_ref(), json_instance.strip.as_ref()) {
            (Some(_), None) => {
                panic!("BP problem")
            }
            (None, Some(json_strip)) => SPInstance::new(items.clone(), json_strip.height).into(),
            (Some(_), Some(_)) => {
                panic!("Both bins and strip packing specified, has to be one or the other")
            }
            (None, None) => panic!("Neither bins or strips specified"),
        };

        let sdr_ins = SdrInstance::new(instance, items, resolution);
        
        sdr_ins
    }

    fn parse_item(&self, json_item: &JsonItem, item_id: usize) -> (Item, usize) {
        let shape = match &json_item.shape {
            JsonShape::Rectangle { width, height } => {
                SimplePolygon::from(AARectangle::new(0.0, 0.0, *width, *height))
            }
            JsonShape::SimplePolygon(sp) => {
                convert_json_simple_poly(sp, self.poly_simpl_config, PolySimplMode::Inflate)
            }
            JsonShape::Polygon(_) => {
                unimplemented!("No support for polygon shapes yet")
            }
            JsonShape::MultiPolygon(_) => {
                unimplemented!("No support for multipolygon shapes yet")
            }
        };

        let item_value = json_item.value.unwrap_or(0);
        let base_quality = json_item.base_quality;

        let allowed_orientations = match json_item.allowed_orientations.as_ref() {
            Some(a_o) => {
                if a_o.is_empty() || (a_o.len() == 1 && a_o[0] == 0.0) {
                    AllowedRotation::None
                } else {
                    AllowedRotation::Discrete(a_o.iter().map(|angle| angle.to_radians()).collect())
                }
            }
            None => AllowedRotation::Continuous,
        };

        let base_item = Item::new(
            item_id,
            shape,
            allowed_orientations,
            base_quality,
            item_value,
            Transformation::empty(),
            self.cde_config.item_surrogate_config,
        );

        let item = match self.center_polygons {
            false => base_item,
            true => {
                let centering_transform = centering_transformation(&base_item.shape);
                pretransform_item(&base_item, &centering_transform.compose())
            }
        };

        (item, json_item.demand as usize)
    }
    
}

fn convert_json_simple_poly(
    s_json_shape: &JsonSimplePoly,
    simpl_config: PolySimplConfig,
    simpl_mode: PolySimplMode,
) -> SimplePolygon {
    let shape = SimplePolygon::new(json_simple_poly_to_points(s_json_shape));

    let shape = match simpl_config {
        PolySimplConfig::Enabled { tolerance } => {
            polygon_simplification::simplify_shape(&shape, simpl_mode, tolerance)
        }
        PolySimplConfig::Disabled => shape,
    };

    shape
}

fn json_simple_poly_to_points(jsp: &JsonSimplePoly) -> Vec<Point> {
    //Strip the last vertex if it is the same as the first one
    let n_vertices = match jsp.0[0] == jsp.0[jsp.0.len() - 1] {
        true => jsp.0.len() - 1,
        false => jsp.0.len(),
    };

    (0..n_vertices).map(|i| Point::from(jsp.0[i])).collect_vec()
}

pub fn pretransform_item(item: &Item, extra_pretransf: &Transformation) -> Item {
    let Item {
        id,
        shape,
        allowed_rotation,
        base_quality,
        value,
        pretransform,
        surrogate_config,
        ..
    } = item;

    Item::new(
        *id,
        shape.transform_clone(extra_pretransf),
        allowed_rotation.clone(),
        *base_quality,
        *value,
        pretransform.clone().transform(extra_pretransf),
        *surrogate_config,
    )
}

pub fn centering_transformation(shape: &SimplePolygon) -> DTransformation {
    let Point(cx, cy) = shape.centroid();
    DTransformation::new(0.0, (-cx, -cy))
}