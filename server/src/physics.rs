use crate::world::*;
use rapier2d::prelude::*;

const WALL_T: f32 = 0.2; // 벽 두께
const BALL_R: f32 = 0.2;
const ROBOT_HX: f32 = 0.25; // 로봇 반폭
const ROBOT_HY: f32 = 0.2;
const RESTITUTION: f32 = 0.85;

pub struct PhysicsWorld {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    gravity: Vector<Real>,
    params: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad: DefaultBroadPhase,
    narrow: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    query: QueryPipeline,
    ball: RigidBodyHandle,
    robots: Vec<RigidBodyHandle>,
    pub score: (u32, u32),
    pub time: f32,
}

impl PhysicsWorld {
    pub fn new_kickoff() -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        let hw = FIELD_W / 2.0;
        let hh = FIELD_H / 2.0;

        // 벽 4개 (고정)
        for (hx, hy, x, y) in [
            (hw, WALL_T, 0.0, hh),
            (hw, WALL_T, 0.0, -hh),
            (WALL_T, hh, hw, 0.0),
            (WALL_T, hh, -hw, 0.0),
        ] {
            colliders.insert(
                ColliderBuilder::cuboid(hx, hy)
                    .translation(vector![x, y])
                    .restitution(RESTITUTION)
                    .build(),
            );
        }

        // 공 (동적)
        let ball = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(vector![0.0, 0.0])
                .linear_damping(0.4)
                .build(),
        );
        colliders.insert_with_parent(
            ColliderBuilder::ball(BALL_R).restitution(RESTITUTION).build(),
            ball,
            &mut bodies,
        );

        // 로봇 2대
        let mut robots = Vec::new();
        for (x, rot) in [(-3.0f32, 0.0f32), (3.0, std::f32::consts::PI)] {
            let rb = bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(vector![x, 0.0])
                    .rotation(rot)
                    .linear_damping(2.0)
                    .angular_damping(4.0)
                    .build(),
            );
            colliders.insert_with_parent(
                ColliderBuilder::cuboid(ROBOT_HX, ROBOT_HY).build(),
                rb,
                &mut bodies,
            );
            robots.push(rb);
        }

        PhysicsWorld {
            bodies,
            colliders,
            gravity: vector![0.0, 0.0],
            params: IntegrationParameters {
                dt: DT,
                ..Default::default()
            },
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad: DefaultBroadPhase::new(),
            narrow: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            query: QueryPipeline::new(),
            ball,
            robots,
            score: (0, 0),
            time: 0.0,
        }
    }

    pub fn snapshot(&self) -> GameState {
        let b = &self.bodies[self.ball];
        let ball = BallState {
            pos: to_vec2(b.translation()),
            vel: to_vec2(b.linvel()),
        };
        let robots = self
            .robots
            .iter()
            .enumerate()
            .map(|(i, h)| {
                let rb = &self.bodies[*h];
                RobotState {
                    id: if i == 0 { Team::Blue } else { Team::Red },
                    pos: to_vec2(rb.translation()),
                    rot: rb.rotation().angle(), // rapier가 정규화된 각도 반환
                    vel: to_vec2(rb.linvel()),
                }
            })
            .collect();
        GameState {
            robots,
            ball,
            score: self.score,
            time: self.time,
        }
    }
}

fn to_vec2(v: &Vector<Real>) -> Vec2 {
    Vec2 { x: v.x, y: v.y }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kickoff_world_has_ball_and_two_robots_in_bounds() {
        let w = PhysicsWorld::new_kickoff();
        let s = w.snapshot();
        assert_eq!(s.robots.len(), 2);
        assert_eq!(s.ball.pos, Vec2 { x: 0.0, y: 0.0 });
        // 경계 안
        assert!(s.ball.pos.x.abs() <= FIELD_W / 2.0);
    }
}
