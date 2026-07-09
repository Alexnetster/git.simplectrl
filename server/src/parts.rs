//! 파츠·스탯 데이터 모델 + 카탈로그 + 로드아웃 집계 (순수 로직, 결정성 안전).
//! `HashMap`은 파츠/프리셋 조회·집계(합산)에만 쓰여 순서 무관 → sim/스냅샷 경로 무순회.

use std::collections::HashMap;

#[derive(Clone, Copy, Debug, Default)]
pub struct StatSet {
    pub max_speed: f32,
    pub accel: f32,
    pub turn_rate: f32,
    pub mass: f32,
    // 정의만(Plan 3b/4에서 사용):
    pub kick_power: f32,
    pub attack: f32,
    pub defense: f32,
    pub hp: f32,
}

impl StatSet {
    fn add(&mut self, o: &StatSet) {
        self.max_speed += o.max_speed;
        self.accel += o.accel;
        self.turn_rate += o.turn_rate;
        self.mass += o.mass;
        self.kick_power += o.kick_power;
        self.attack += o.attack;
        self.defense += o.defense;
        self.hp += o.hp;
    }
}

/// 기존 하드코딩(THRUST=6/TURN_RATE=3)과 등가인 기본 스탯.
/// mass는 콜라이더 밀도 유래 질량에 가산되므로 0=no-op(기존 거동 보존).
pub fn default_stats() -> StatSet {
    StatSet {
        max_speed: 10.0,
        accel: 6.0,
        turn_rate: 3.0,
        mass: 0.0,
        ..Default::default()
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Slot {
    Head,
    Neck,
    Body,
    ForelegL,
    ForelegR,
    HindlegL,
    HindlegR,
    Tail,
}

impl Slot {
    pub fn as_str(&self) -> &'static str {
        match self {
            Slot::Head => "head",
            Slot::Neck => "neck",
            Slot::Body => "body",
            Slot::ForelegL => "foreleg_l",
            Slot::ForelegR => "foreleg_r",
            Slot::HindlegL => "hindleg_l",
            Slot::HindlegR => "hindleg_r",
            Slot::Tail => "tail",
        }
    }
}

#[derive(Clone)]
pub struct Part {
    pub id: &'static str,
    pub slot: Slot,
    pub stats: StatSet,
}

/// 파츠 id 목록으로 표현한 로드아웃.
pub struct Loadout {
    pub parts: Vec<&'static str>,
}

pub struct Catalog {
    pub parts: HashMap<&'static str, Part>,
    pub presets: HashMap<&'static str, Loadout>,
}

/// 데이터 주도 카탈로그(개발자 배포). 값은 밸런싱 대상 초기값.
pub fn catalog() -> Catalog {
    let mut parts = HashMap::new();
    let mut add = |id, slot, s: StatSet| {
        parts.insert(id, Part { id, slot, stats: s });
    };
    // 이동 스탯은 다리가 주도(좌/우 한 쌍씩 기여). 몸통은 mass/hp/defense,
    // 목은 turn_rate. 로봇은 4족: 앞다리 L/R + 뒷다리 L/R.
    // 프리셋 총합이 default_stats(max_speed≈10/accel≈6/turn_rate=3)에 준하도록 구성.
    add(
        "body-std",
        Slot::Body,
        StatSet {
            mass: 1.0,
            hp: 40.0,
            defense: 6.0,
            ..Default::default()
        },
    );
    add(
        "body-light",
        Slot::Body,
        StatSet {
            mass: 0.7,
            hp: 30.0,
            defense: 4.0,
            ..Default::default()
        },
    );
    // 스피드형 뒷다리(빠르지만 가속 낮음) — 좌/우
    let hind_speed = StatSet {
        max_speed: 5.5,
        accel: 2.0,
        ..Default::default()
    };
    add("hind-speed-l", Slot::HindlegL, hind_speed);
    add("hind-speed-r", Slot::HindlegR, hind_speed);
    // 파워형 뒷다리(가속 높지만 최고속 낮음) — 좌/우
    let hind_power = StatSet {
        max_speed: 4.0,
        accel: 3.5,
        ..Default::default()
    };
    add("hind-power-l", Slot::HindlegL, hind_power);
    add("hind-power-r", Slot::HindlegR, hind_power);
    // 표준 앞다리 — 좌/우
    let fore_std = StatSet {
        accel: 1.0,
        attack: 2.5,
        ..Default::default()
    };
    add("fore-std-l", Slot::ForelegL, fore_std);
    add("fore-std-r", Slot::ForelegR, fore_std);
    add(
        "neck-std",
        Slot::Neck,
        StatSet {
            turn_rate: 3.0,
            ..Default::default()
        },
    );
    add("head-std", Slot::Head, StatSet { ..Default::default() });
    add("tail-std", Slot::Tail, StatSet { ..Default::default() });

    let mut presets = HashMap::new();
    presets.insert(
        "striker",
        Loadout {
            parts: vec![
                "head-std",
                "neck-std",
                "body-light",
                "fore-std-l",
                "fore-std-r",
                "hind-speed-l",
                "hind-speed-r",
                "tail-std",
            ],
        },
    );
    presets.insert(
        "guard",
        Loadout {
            parts: vec![
                "head-std",
                "neck-std",
                "body-std",
                "fore-std-l",
                "fore-std-r",
                "hind-power-l",
                "hind-power-r",
                "tail-std",
            ],
        },
    );

    Catalog { parts, presets }
}

/// 프리셋 id의 총 스탯 = 부위 기여 합.
pub fn aggregate(cat: &Catalog, preset: &str) -> StatSet {
    let mut s = StatSet::default();
    if let Some(lo) = cat.presets.get(preset) {
        for pid in &lo.parts {
            if let Some(p) = cat.parts.get(pid) {
                s.add(&p.stats);
            }
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preset_aggregates_part_stats_and_presets_differ() {
        let cat = catalog();
        let striker = aggregate(&cat, "striker");
        let guard = aggregate(&cat, "guard");
        // 집계는 부위 기여 합
        assert!(striker.max_speed > 0.0 && striker.accel > 0.0);
        // 프리셋이 서로 다르다(비대칭)
        assert!(striker.max_speed != guard.max_speed || striker.accel != guard.accel);
    }
}
