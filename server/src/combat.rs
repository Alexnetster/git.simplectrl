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
}
