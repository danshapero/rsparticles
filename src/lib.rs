extern crate nalgebra as na;
use na::Vector2;

pub struct Particle {
    position: Vector2<f64>,
    velocity: Vector2<f64>,
}

pub fn collision_parameters(
    particle1: &Particle,
    particle2: &Particle,
    radius: f64,
) -> Option<(f64, Particle, Particle)> {
    let dx = particle1.position - particle2.position;
    let dv = particle1.velocity - particle2.velocity;
    let dx_dv = dx.dot(&dv);
    if dx_dv > 0.0 {
        return None;
    }

    let dx2 = dx.dot(&dx);
    let dv2 = dv.dot(&dv);
    let discriminant = dx_dv * dx_dv - dv2 * (dx2 - 4.0 * radius * radius);
    if discriminant < 0.0 {
        return None;
    }

    let time = (-dx_dv - f64::sqrt(discriminant)) / dv2;

    let x1_collision = particle1.position + time * particle1.velocity;
    let x2_collision = particle2.position + time * particle2.velocity;
    let dx = x1_collision - x2_collision;
    let dx_dv = dx.dot(&dv);
    let velocity_change = -dx_dv / (4.0 * radius * radius) * dx;
    let particle1_collision = Particle {
        position: x1_collision,
        velocity: particle1.velocity + velocity_change,
    };
    let particle2_collision = Particle {
        position: x2_collision,
        velocity: particle2.velocity - velocity_change,
    };
    return Some((time, particle1_collision, particle2_collision));
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
        let particle1 = Particle {
            position: x1,
            velocity: v1,
        };
        let particle2 = Particle {
            position: x2,
            velocity: v2,
        };
        let radius = 1.0;
        let result = collision_parameters(&particle1, &particle2, radius);
        assert!(result.is_some());

        let (time, particle1_coll, particle2_coll) = result.unwrap();
        let momentum = particle1.velocity + particle2.velocity;
        let momentum_coll = particle1_coll.velocity + particle2_coll.velocity;
        let epsilon = 1.0e-6;
        assert!((momentum - momentum_coll).norm() < epsilon);
        let time_expected = 9.0;
        assert!(f64::abs(time - time_expected) < epsilon);
    }

    #[test]
    fn no_collision() {
        let radius = 1.0;
        let epsilon = 0.1;
        let x1 = Vector2::new(-10.0, -radius - epsilon);
        let x2 = Vector2::new(10.0, radius + epsilon);
        let v1 = Vector2::new(1.0, 0.0);
        let v2 = Vector2::new(-1.0, 0.0);
        let particle1 = Particle {
            position: x1,
            velocity: v1,
        };
        let particle2 = Particle {
            position: x2,
            velocity: v2,
        };
        let result = collision_parameters(&particle1, &particle2, radius);
        assert!(result.is_none());
    }
}
