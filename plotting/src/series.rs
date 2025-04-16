use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Series {
    pub xname: Option<String>,
    pub yname: Option<String>,
    pub xdata: Vec<f64>,
    pub ydata: Vec<f64>,
}

impl Series {
    pub fn new(xdata: Vec<f64>, ydata: Vec<f64>) -> Self {
        Self {
            xname: None,
            yname: None,
            xdata,
            ydata,
        }
    }

    pub fn set_x_name(&mut self, name: String) {
        self.xname = Some(name);
    }

    pub fn set_y_name(&mut self, name: String) {
        self.yname = Some(name);
    }
}
