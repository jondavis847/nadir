use iced::Point;

#[derive(Debug)]
pub struct SeriesMap {
    pub map: Vec<Series>,
    pub xmax: f64,
    pub xmin: f64,
    pub ymax: f64,
    pub ymin: f64,
    pub axes: (usize, usize), // which axes to plot on, (row,column)
}

impl SeriesMap {
    pub fn new(axes: (usize, usize)) -> Self {
        Self {
            map: Vec::new(),
            xmax: -f64::INFINITY,
            xmin: f64::INFINITY,
            ymax: -f64::INFINITY,
            ymin: f64::INFINITY,
            axes,
        }
    }

    pub fn insert(&mut self, series: Series) {
        if series.xmin < self.xmin {
            self.xmin = series.xmin;
        }
        if series.xmax > self.xmax {
            self.xmax = series.xmax;
        }
        if series.ymin < self.ymin {
            self.ymin = series.ymin;
        }
        if series.ymax > self.ymax {
            self.ymax = series.ymax;
        }

        self.map.push(series);
    }
}

#[derive(Debug)]
pub struct Series {
    pub x_name: String,
    pub y_name: String,
    pub points: Vec<PlotPoint>,
    xmax: f64,
    ymax: f64,
    xmin: f64,
    ymin: f64,
}

impl Series {
    pub fn new(x_name: String, x: Vec<f64>, y_name: String, y: Vec<f64>) -> Self {
        let xmin = x.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let ymin = y.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let xmax = x.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
        let ymax = y.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
        let points = x
            .into_iter()
            .zip(y)
            .map(|(x, y)| PlotPoint::new(x, y, true))
            .collect();

        Self {
            x_name,
            y_name,
            points,
            xmin,
            ymin,
            xmax,
            ymax,
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }
}

#[derive(Debug)]
pub struct PlotPoint {
    pub canvas_position: Point,
    pub data: iced::Point<f64>,
    pub datatip: bool, // only show datatips for legitimate sim data, interp points dont show datatips for example
}

impl PlotPoint {
    pub fn new(x: f64, y: f64, datatip: bool) -> Self {
        Self {
            canvas_position: Point::new(0.0, 0.0), // to be updated later
            data: Point::<f64>::new(x, y),
            datatip,
        }
    }
}
