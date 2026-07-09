//! 전투/데미지 순수 로직 (결정적, I/O 없음). 충돌 감지는 physics.rs의 경계.
//! attack/defense는 **로봇 총합**(3b) — 부위별 세분화·취약도 항은 Plan 3c 여지.

use crate::parts::StatSet;

/// 상호 데미지 한쪽 산출: impact × (공격 / (방어+1)) × 계수. 결정적·비음수.
///
/// `impact`는 ADR-009의 접촉 임펄스를 **상대 linvel 크기**로 간소화한 근사(3b 의도적 간소화).
/// 진짜 임펄스는 `ContactForceEvent`(CONTACT_FORCE_EVENTS + threshold)로 3c/튜닝에서.
/// `defense + 1`로 0방어 폭주를 방지한다.
pub fn damage_on_contact(attacker: &StatSet, defender: &StatSet, impact: f32) -> f32 {
    const K: f32 = 1.0;
    let atk = attacker.attack.max(0.0);
    let def = defender.defense.max(0.0) + 1.0;
    (impact.max(0.0) * (atk / def) * K).max(0.0)
}

/// 파손 다운 지속 틱(3초 @60Hz). 튜닝 대상.
const REPAIR_TICKS: u32 = 180;

/// 로봇 1대의 부위별 HP + 파손 다운/리페어 타이머 (결정적 순수 상태).
/// 어떤 부위든 HP가 0에 닿으면 파손 다운 → 타이머 소진 시 **전체 부위** 리페어.
pub struct CombatState {
    max: Vec<f32>,
    hp: Vec<f32>,
    down_timer: u32,
}

impl CombatState {
    pub fn new(max_hp: &[f32]) -> Self {
        Self {
            max: max_hp.to_vec(),
            hp: max_hp.to_vec(),
            down_timer: 0,
        }
    }

    pub fn broken(&self) -> bool {
        self.down_timer > 0
    }

    #[cfg(test)]
    pub fn repair_ticks(&self) -> u32 {
        REPAIR_TICKS
    }

    /// 리페어까지 남은 초(스냅샷 `down.repair_in`용). 다운 아니면 0.
    pub fn repair_in(&self) -> f32 {
        self.down_timer as f32 * crate::world::DT
    }

    pub fn part_count(&self) -> usize {
        self.hp.len()
    }

    pub fn hp_ratio(&self, i: usize) -> f32 {
        if self.max[i] > 0.0 {
            self.hp[i] / self.max[i]
        } else {
            1.0
        }
    }

    /// 모든 부위 중 최소 HP비율(테스트/디버프 판정용).
    #[cfg(test)]
    pub fn hp_ratio_min(&self) -> f32 {
        (0..self.hp.len())
            .map(|i| self.hp_ratio(i))
            .fold(1.0_f32, f32::min)
    }

    pub fn apply_damage(&mut self, part: usize, dmg: f32) {
        if self.broken() {
            return;
        }
        self.hp[part] = (self.hp[part] - dmg).max(0.0);
        if self.hp.iter().any(|&h| h <= 0.0) {
            self.down_timer = REPAIR_TICKS;
        }
    }

    /// 다운 중 매 tick 호출. 타이머 소진 시 전체 리페어.
    pub fn tick_down(&mut self) {
        if self.down_timer > 0 {
            self.down_timer -= 1;
            if self.down_timer == 0 {
                self.hp = self.max.clone();
            }
        }
    }

    /// 강제 파손 다운(테스트 전용).
    #[cfg(test)]
    pub fn force_down(&mut self) {
        self.down_timer = REPAIR_TICKS;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn damage_scales_with_impact_and_attack_over_defense() {
        // 공격력↑ 또는 impact↑ → 데미지↑, 방어력↑ → 데미지↓
        let atk = StatSet {
            attack: 10.0,
            ..Default::default()
        };
        let def_low = StatSet {
            defense: 2.0,
            ..Default::default()
        };
        let def_high = StatSet {
            defense: 8.0,
            ..Default::default()
        };
        let d_low = damage_on_contact(&atk, &def_low, 1.0);
        let d_high = damage_on_contact(&atk, &def_high, 1.0);
        let d_big = damage_on_contact(&atk, &def_low, 2.0);
        assert!(d_low > d_high, "방어 높으면 데미지 감소");
        assert!(d_big > d_low, "impact 크면 데미지 증가");
        assert!(d_low >= 0.0);
    }

    #[test]
    fn part_hp_depletes_and_triggers_down_then_repairs() {
        let mut cs = CombatState::new(&[40.0, 30.0]); // 2 부위
        assert!(!cs.broken());
        cs.apply_damage(0, 100.0); // 부위0 과다 피해
        assert!(cs.broken(), "부위 HP 0 → 파손 다운");
        // 다운 중 추가 피해는 무시(재트리거/중첩 없음)
        cs.apply_damage(1, 100.0);
        // 다운 지속 후 리페어
        for _ in 0..(cs.repair_ticks()) {
            cs.tick_down();
        }
        assert!(!cs.broken(), "일정 시간 뒤 전체 리페어");
        assert!(cs.hp_ratio(0) > 0.99, "리페어 시 부위0 HP 복구");
        assert!(cs.hp_ratio(1) > 0.99, "리페어 시 전체 부위 복구");
    }

    #[test]
    fn zero_damage_does_not_trigger_down() {
        // 데미지 0(예: attack=0 로봇)은 파손 다운을 유발하지 않는다.
        let mut cs = CombatState::new(&[20.0, 20.0]);
        cs.apply_damage(0, 0.0);
        assert!(!cs.broken());
        assert!(cs.hp_ratio_min() > 0.99);
    }
}
