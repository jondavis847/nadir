// use std::collections::HashMap;

// mod application;
// // mod axes;
// // mod axis;
// mod canvas;
// mod figure;
// // mod line;
// // mod series;
// mod theme;

// use crate::figure::Figure;

// #[derive(Debug, Default)]
// pub struct Identifier {
//     current: u32,
// }

// impl Identifier {
//     fn next(&mut self) -> u32 {
//         let value = self.current;
//         self.current += 1;
//         value
//     }
// }

// #[derive(Debug, Default)]
// pub struct PlotManager {
//     identifier: Identifier,
//     figures: HashMap<u32, Figure>,
// }

// impl PlotManager {
//     pub fn new_figure(&mut self) {
//         let id = self.identifier.next();
//         let fig = Figure::new(id);
//         self.figures.insert(id, fig);
//     }
// }

// pub fn main() {

//     // let root_path = provided_path
//     //     .unwrap_or_else(|| std::env::current_dir().expect("could not get current directory"));

//     // let mut first_pass = true;

//     // // axes loop
//     // loop {
//     //     let mut map = if first_pass {
//     //         first_pass = false;
//     //         SeriesMap::new((0, 0))
//     //     } else {
//     //         let this_location = loop {
//     //             let row: usize =
//     //                 prompt_positive_integer("enter axes layout row (0 based indexing!)");
//     //             let column: usize =
//     //                 prompt_positive_integer("enter axes layout column (0 based indexing!)");

//     //             let location = (row, column);

//     //             if !series
//     //                 .iter()
//     //                 .any(|entry: &SeriesMap| entry.axes == location)
//     //             {
//     //                 break location; // Found a valid location
//     //             } else {
//     //                 println!("Error: axes location {:?} is taken", location);
//     //             }
//     //         };
//     //         SeriesMap::new(this_location)
//     //     };
//     //     // series loop
//     //     loop {
//     //         let mut path = select_x_source(&root_path);
//     //         while path.is_dir() {
//     //             path = select_x_source(&path);
//     //         }
//     //         let x_name = select_x_data(&path);
//     //         let x_data = extract_column(&path, &x_name);

//     //         let mut path = select_y_source(&root_path);
//     //         while path.is_dir() {
//     //             path = select_y_source(&path);
//     //         }
//     //         let y_names = select_y_data(&path);
//     //         for y_name in y_names {
//     //             let y_data = extract_column(&path, &y_name);
//     //             map.insert(Series::new(x_name.clone(), x_data.clone(), y_name, y_data));
//     //         }

//     //         // Ask the user if they want to add another series
//     //         let add_another = Confirm::new("add another line?")
//     //             .with_default(false)
//     //             .prompt()
//     //             .unwrap_or(false);
//     //         if !add_another {
//     //             break;
//     //         }
//     //     }
//     //     series.push(map);
//     //     // Ask the user if they want to add another axes
//     //     let add_another = Confirm::new("add another axes?")
//     //         .with_default(false)
//     //         .prompt()
//     //         .unwrap_or(false);
//     //     if !add_another {
//     //         break;
//     //     }
//     // }
// }
