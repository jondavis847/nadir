use iced::Point;

use super::PlotErrors;

#[derive(Debug, Clone)]
pub struct Series {
    pub xname: Option<String>,
    pub yname: Option<String>,
    pub points: Vec<PlotPoint>,
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
}

impl Series {
    pub fn new(xdata: Vec<f64>, ydata: Vec<f64>) -> Result<Self, PlotErrors> {
        if xdata.len() != ydata.len() {
            return Err(PlotErrors::DataSizeMismatch);
        }

        let xmin = xdata.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let xmax = xdata.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
        let ymin = ydata.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let ymax = ydata.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));

        let points = xdata
            .iter()
            .zip(ydata)
            .map(|(x, y)| PlotPoint::new(*x, y))
            .collect();
        Ok(Self {
            xname: None,
            yname: None,
            points,
            xmin,
            xmax,
            ymin,
            ymax,
        })
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }
    pub fn set_x_name(&mut self, name: String) {
        self.xname = Some(name);
    }

    pub fn set_y_name(&mut self, name: String) {
        self.yname = Some(name);
    }
}

#[derive(Debug, Clone)]
pub struct PlotPoint {
    pub canvas_position: Point,
    pub data: Point<f64>,
}

impl PlotPoint {
    fn new(x: f64, y: f64) -> Self {
        Self {
            data: Point::new(x, y),
            canvas_position: Point::ORIGIN, // updated later
        }
    }
}
