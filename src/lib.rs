extern crate nalgebra as na;
use na::Vector2;

pub struct Particle {
    position: Vector2<f64>,
    velocity: Vector2<f64>,
}

pub fn collision_time(particle1: Particle, particle2: Particle, radius: f64) -> Option<f64> {
    let dx = particle1.position - particle2.position;
    let dv = particle1.velocity - particle2.velocity;
    let dx_dv = dx.dot(&dv);
    let dx2 = dx.dot(&dv);
    let dv2 = dv.dot(&dv);
    let discriminant = dx_dv * dx_dv - dv2 * (dx2 - 4.0 * radius * radius);
    if discriminant < 0.0 {
        return None;
    }

    return Some((f64::sqrt(discriminant) - dx_dv) / dv2);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_head_on_collision() {
        let x1 = Vector2::new(-10.0, 0.0);
        let x2 = Vector2::new(10.0, 0.0);
        let v1 = Vector2::new(1.0, 0.0);
        let v2 = Vector2::new(-1.0, 0.0);
        let particle1 = Particle{position: x1, velocity: v1};
        let particle2 = Particle{position: x2, velocity: v2};
        let radius = 1.0;
        let result = collision_time(particle1, particle2, radius);
        assert!(result.is_some());
    }

    #[test]
    fn no_collision() {
        let radius = 1.0;
        let epsilon = 0.1;
        let x1 = Vector2::new(-10.0, -2.0 * radius - epsilon);
        let x2 = Vector2::new(10.0, 2.0 * radius + epsilon);
        let v1 = Vector2::new(1.0, 0.0);
        let v2 = Vector2::new(-1.0, 0.0);
        let particle1 = Particle{position: x1, velocity: v1};
        let particle2 = Particle{position: x2, velocity: v2};
        let result = collision_time(particle1, particle2, radius);
        assert!(result.is_none());
    }
}
