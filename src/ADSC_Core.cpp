#include "ADSC_Core.h"
#include <iostream>

uint32_t ADSC_Core::compute_crc(double v, uint32_t ver) {
    uint32_t hash = 2166136261u;
    uint64_t data;
    std::memcpy(&data, &v, sizeof(double));
    for (int i = 0; i < 8; ++i) {
        hash ^= (data >> (i * 8)) & 0xFF;
        hash *= 16777619u;
    }
    for (int i = 0; i < 4; ++i) {
        hash ^= (ver >> (i * 8)) & 0xFF;
        hash *= 16777619u;
    }
    return hash;
}

bool ADSC_Core::verify_crc(const FuelCell& c) {
    return c.crc == compute_crc(c.value, c.version);
}

bool ADSC_Core::nearly_equal(double a, double b, double eps) {
    return std::abs(a - b) < eps * std::max({1.0, std::abs(a), std::abs(b)});
}

double ADSC_Core::median(double a, double b, double c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

// --- 初期化 ---
ADSC_Core::ADSC_Core() {
    FuelCell init;
    init.value = 7.2;
    init.version = 1;
    init.crc = compute_crc(init.value, init.version);

    for (int i = 0; i < 3; ++i) {
        fuel_cells[i].store(init, std::memory_order_relaxed);
    }

    mass_total = 20.0 + 7.2;
    inertia_tensor = Eigen::Matrix3d::Identity() * 0.55;
}

// --- 燃料操作 ---
void ADSC_Core::set_fuel(double f) {
    if (f < 0.0) f = 0.0;

    FuelCell current = fuel_cells[0].load(std::memory_order_acquire);
    uint32_t new_version = current.version + 1;
    if (new_version == 0) new_version = 1;

    FuelCell new_cell{f, new_version, compute_crc(f, new_version)};
    for (int i = 0; i < 3; ++i) {
        fuel_cells[i].store(new_cell, std::memory_order_release);
    }
}

double ADSC_Core::get_fuel() {
    FuelCell c[3];
    for (int i = 0; i < 3; ++i) {
        c[i] = fuel_cells[i].load(std::memory_order_acquire);
    }

    bool valid[3];
    for (int i = 0; i < 3; ++i) valid[i] = verify_crc(c[i]);

    uint32_t v[3] = {c[0].version, c[1].version, c[2].version};
    uint32_t v_major = 0;
    if (valid[0] && valid[1] && v[0] == v[1]) v_major = v[0];
    else if (valid[0] && valid[2] && v[0] == v[2]) v_major = v[0];
    else if (valid[1] && valid[2] && v[1] == v[2]) v_major = v[1];
    else for (int i = 0; i < 3; ++i) if (valid[i]) v_major = std::max(v_major, v[i]);

    double vals[3]; int count = 0;
    for (int i = 0; i < 3; ++i)
        if (valid[i] && c[i].version == v_major)
            vals[count++] = c[i].value;

    double result;
    if (count >= 2) result = (count==2) ? (vals[0]+vals[1])/2.0 : median(vals[0],vals[1],vals[2]);
    else {
        double fallback_vals[3]; int fallback_count=0;
        for(int i=0;i<3;i++) if(valid[i]) fallback_vals[fallback_count++]=c[i].value;
        if(fallback_count==3) result=median(fallback_vals[0],fallback_vals[1],fallback_vals[2]);
        else if(fallback_count==2) result=(fallback_vals[0]+fallback_vals[1])/2.0;
        else if(fallback_count==1) result=fallback_vals[0];
        else result=0.0;
    }

    FuelCell repaired{result,v_major,compute_crc(result,v_major)};
    for(int i=0;i<3;i++){
        FuelCell ci=fuel_cells[i].load(std::memory_order_acquire);
        if(!valid[i]||ci.version!=v_major||!nearly_equal(ci.value,result))
            fuel_cells[i].store(repaired,std::memory_order_release);
    }

    return result;
}

// --- デブリ捕獲処理（実機向けパラメータ反映） ---
void ADSC_Core::post_capture_stabilization(bool captured, double debris_mass, const Eigen::Vector3d& v_rel_vec) {
    if(!captured || v_rel_vec.norm() > max_v_rel) return;
    mass_total += debris_mass;

    // 必要に応じてゲイン・線形化更新
    // update_gains_and_linearization();
    // log_incident("CAPTURE","Debris mass",debris_mass);
}
