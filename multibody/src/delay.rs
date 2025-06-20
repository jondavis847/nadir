use std::collections::VecDeque;

#[derive(Debug, Clone, Copy)]
pub enum InterpolationMethod {
    Linear,
    CubicHermite,
    CubicSpline,
    Lagrange,
}

#[derive(Debug, Clone, Copy)]
struct DelayEntry {
    time: f64,
    value: f64,
}

#[derive(Debug)]
pub struct DelayedValue {
    pub delay: f64,
    history: VecDeque<DelayEntry>,
    interpolation_method: InterpolationMethod,
}

impl DelayedValue {
    pub fn new(delay: f64) -> Self {
        Self {
            delay,
            history: VecDeque::new(),
            interpolation_method: InterpolationMethod::Linear,
        }
    }

    pub fn with_interpolation(mut self, method: InterpolationMethod) -> Self {
        self.interpolation_method = method;
        self
    }

    pub fn update(&mut self, time: f64, value: f64) {
        self.history.push_back(DelayEntry { time, value });

        // Keep extra history based on interpolation method
        let (history_multiplier, min_history_points) = match self.interpolation_method {
            InterpolationMethod::Linear => (5.0, 3), // Need at least 2 for interpolation + 1 buffer
            InterpolationMethod::CubicHermite => (10.0, 6), // Need 4 for algorithm + 2 buffer
            InterpolationMethod::CubicSpline => (10.0, 6), // Similar to Hermite
            InterpolationMethod::Lagrange => (15.0, 8), // Need up to 4 for algorithm + 4 buffer
        };

        // Only prune if we have sufficient history AND the data is old enough
        if self.history.len() > min_history_points {
            let cutoff_time = time - self.delay * history_multiplier;
            while let Some(entry) = self.history.front() {
                if entry.time < cutoff_time && self.history.len() > min_history_points {
                    self.history.pop_front();
                } else {
                    break;
                }
            }
        }
    }

    pub fn get_delayed_reading(&self, current_time: f64) -> f64 {
        if self.history.is_empty() {
            return 0.0;
        }

        let target_time = current_time - self.delay;

        // If target time is before our earliest data, return earliest value
        if let Some(first) = self.history.front() {
            if target_time <= first.time {
                return first.value;
            }
        }

        // If target time is after our latest data, return latest value
        if let Some(last) = self.history.back() {
            if target_time >= last.time {
                return last.value;
            }
        }

        self.interpolate_at_time(target_time)
    }

    fn interpolate_at_time(&self, target_time: f64) -> f64 {
        match self.interpolation_method {
            InterpolationMethod::Linear => self.linear_interpolate(target_time),
            InterpolationMethod::CubicHermite => self.hermite_interpolate(target_time),
            InterpolationMethod::CubicSpline => self.cubic_spline_interpolate(target_time),
            InterpolationMethod::Lagrange => self.lagrange_interpolate(target_time),
        }
    }

    fn linear_interpolate(&self, target_time: f64) -> f64 {
        if self.history.len() < 2 {
            return self
                .history
                .back()
                .map(|e| e.value)
                .unwrap_or(0.0);
        }

        let (before_idx, after_idx) = self.find_bracket_indices(target_time);

        match (before_idx, after_idx) {
            (Some(before), Some(after)) if before != after => {
                let before = &self.history[before];
                let after = &self.history[after];
                let alpha = (target_time - before.time) / (after.time - before.time);
                before.value + alpha * (after.value - before.value)
            }
            (Some(idx), _) => self.history[idx].value,
            _ => {
                // return the closest available value
                if target_time < self.history.front().unwrap().time {
                    self.history.front().unwrap().value
                } else {
                    self.history.back().unwrap().value
                }
            }
        }
    }

    fn hermite_interpolate(&self, target_time: f64) -> f64 {
        if self.history.len() < 4 {
            return self.linear_interpolate(target_time);
        }

        // Find the 4 points around target_time for cubic Hermite
        let center_idx = self.find_closest_index(target_time);
        let indices = self.get_hermite_indices(center_idx);

        if indices.len() != 4 {
            return self.linear_interpolate(target_time);
        }

        let points: Vec<(f64, f64)> = indices
            .iter()
            .map(|&i| (self.history[i].time, self.history[i].value))
            .collect();

        self.cubic_hermite_interpolation(&points, target_time)
    }

    fn cubic_hermite_interpolation(&self, points: &[(f64, f64)], target_time: f64) -> f64 {
        // Use points[1] and points[2] as the main interval
        let (t1, y1) = points[1];
        let (t2, y2) = points[2];

        // Calculate derivatives at endpoints using neighboring points
        let m1 = self.calculate_derivative_at_point(points, 1);
        let m2 = self.calculate_derivative_at_point(points, 2);

        // Normalize time to [0, 1]
        let h = t2 - t1;
        let t = (target_time - t1) / h;

        // Hermite basis functions
        let h00 = 2.0 * t.powi(3) - 3.0 * t.powi(2) + 1.0;
        let h10 = t.powi(3) - 2.0 * t.powi(2) + t;
        let h01 = -2.0 * t.powi(3) + 3.0 * t.powi(2);
        let h11 = t.powi(3) - t.powi(2);

        // Interpolated value
        h00 * y1 + h10 * h * m1 + h01 * y2 + h11 * h * m2
    }

    fn calculate_derivative_at_point(&self, points: &[(f64, f64)], index: usize) -> f64 {
        match index {
            0 => {
                // Forward difference
                (points[1].1 - points[0].1) / (points[1].0 - points[0].0)
            }
            i if i == points.len() - 1 => {
                // Backward difference
                (points[i].1 - points[i - 1].1) / (points[i].0 - points[i - 1].0)
            }
            i => {
                // Central difference (more accurate)
                (points[i + 1].1 - points[i - 1].1) / (points[i + 1].0 - points[i - 1].0)
            }
        }
    }

    fn lagrange_interpolate(&self, target_time: f64) -> f64 {
        if self.history.len() < 3 {
            return self.linear_interpolate(target_time);
        }

        // Use up to 4 points for cubic Lagrange
        let n_points = (self.history.len()).min(4);
        let center_idx = self.find_closest_index(target_time);
        let indices = self.get_centered_indices(center_idx, n_points);

        let mut result = 0.0;

        for (i, &idx_i) in indices.iter().enumerate() {
            let (t_i, y_i) = (
                self.history[idx_i].time,
                self.history[idx_i].value,
            );

            // Calculate Lagrange basis polynomial L_i(t)
            let mut li = 1.0;
            for (j, &idx_j) in indices.iter().enumerate() {
                if i != j {
                    let t_j = self.history[idx_j].time;
                    li *= (target_time - t_j) / (t_i - t_j);
                }
            }

            result += y_i * li;
        }

        result
    }

    fn cubic_spline_interpolate(&self, target_time: f64) -> f64 {
        // Simplified cubic spline (you could implement full spline with second derivatives)
        // For now, falls back to Hermite
        self.hermite_interpolate(target_time)
    }

    // Helper functions
    fn find_bracket_indices(&self, target_time: f64) -> (Option<usize>, Option<usize>) {
        let mut before_idx = None;
        let mut after_idx = None;

        for (i, entry) in self.history.iter().enumerate() {
            if entry.time <= target_time {
                before_idx = Some(i);
            }
            if entry.time >= target_time && after_idx.is_none() {
                after_idx = Some(i);
                break;
            }
        }

        (before_idx, after_idx)
    }

    fn find_closest_index(&self, target_time: f64) -> usize {
        self.history
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                (a.time - target_time)
                    .abs()
                    .partial_cmp(&(b.time - target_time).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    fn get_hermite_indices(&self, center_idx: usize) -> Vec<usize> {
        let len = self.history.len();
        if len < 4 {
            return Vec::new();
        }

        // Try to get 2 points before and 2 points after (or 1 before, 3 after, etc.)
        let start = if center_idx >= 2 {
            center_idx - 2
        } else {
            0
        };
        let end = (start + 4).min(len);
        let actual_start = if end == len && len >= 4 {
            len - 4
        } else {
            start
        };

        (actual_start..end).collect()
    }

    fn get_centered_indices(&self, center_idx: usize, n_points: usize) -> Vec<usize> {
        let len = self.history.len();
        let half = n_points / 2;

        let start = if center_idx >= half {
            center_idx - half
        } else {
            0
        };
        let end = (start + n_points).min(len);
        let actual_start = if end == len && len >= n_points {
            len - n_points
        } else {
            start
        };

        (actual_start..end).collect()
    }
}
