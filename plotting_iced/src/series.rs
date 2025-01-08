use iced::Point;

#[derive(Debug)]
pub struct SeriesMap {
    pub map: Vec<Series>,
    pub xmax: f32,
    pub xmin: f32,
    pub ymax: f32,
    pub ymin: f32,
    pub axes: (usize, usize), // which axes to plot on, (row,column)
}

impl SeriesMap {
    pub fn new(axes: (usize, usize)) -> Self {
        Self {
            map: Vec::new(),
            xmax: -f32::INFINITY,
            xmin: f32::INFINITY,
            ymax: -f32::INFINITY,
            ymin: f32::INFINITY,
            axes,
        }
    }

    pub fn insert(&mut self, series: Series) {
        if let Some((xmin, xmax, ymin, ymax)) = &series.find_boundaries() {
            if xmin < &self.xmin {
                self.xmin = *xmin;
            }
            if xmax > &self.xmax {
                self.xmax = *xmax;
            }
            if ymin < &self.ymin {
                self.ymin = *ymin;
            }
            if ymax > &self.ymax {
                self.ymax = *ymax;
            }

            self.map.push(series);
        } else {
            panic!("Could not find data boundaries. Check the data is not corrupted.")
        };
    }
}

#[derive(Debug)]
pub struct Series {
    //pub x_name: String,
    pub y_name: String,
    pub points: Vec<Point>,
}

impl Series {
    pub fn new(_x_name: String, x: Vec<f64>, y_name: String, y: Vec<f64>) -> Self {
        let points = x
            .into_iter()
            .zip(y)
            .map(|(x, y)| Point::new(x as f32, y as f32))
            .collect();

        Self {
            //x_name,
            y_name,
            points,
        }
    }

    pub fn find_boundaries(&self) -> Option<(f32, f32, f32, f32)> {
        if self.points.is_empty() {
            return None;
        }

        let mut xmin = f32::INFINITY;
        let mut xmax = f32::NEG_INFINITY;
        let mut ymin = f32::INFINITY;
        let mut ymax = f32::NEG_INFINITY;

        for point in &self.points {
            if point.x.is_nan() || point.y.is_nan() {
                continue; // Skip points with NaN coordinates
            }
            if point.x < xmin {
                xmin = point.x;
            }
            if point.x > xmax {
                xmax = point.x;
            }
            if point.y < ymin {
                ymin = point.y;
            }
            if point.y > ymax {
                ymax = point.y;
            }
        }

        if xmin == f32::INFINITY
            || xmax == f32::NEG_INFINITY
            || ymin == f32::INFINITY
            || ymax == f32::NEG_INFINITY
        {
            None // All points were NaN
        } else {
            Some((xmin, xmax, ymin, ymax))
        }
    }
}
