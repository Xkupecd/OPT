# OPT – Optimalizácia a optimálne riadenie (9. semester, STU FEI, ÚRK)

Pracovný priečinok k predmetu. Všetky zadania sa riešia v **MATLAB R2024b**
(`C:\Program Files\MATLAB\R2024b\bin\matlab`) doplnením `%TODO` miest v kostre
kódu `optim_course`. Zadania (PDF) obsahujú teóriu, presné signatúry funkcií
a obrázok „Riešenie“, ktorému má výsledok zodpovedať.

## Štruktúra

```
OPT/
├── Prednasky/                    # prednášky (PDF)
│   ├── 0_uvod.pdf
│   ├── 1_funkcie.pdf
│   ├── 2_gradientne_metody.pdf
│   └── 3_kalmanov_filter.pdf
└── Cvicenia/
    ├── Cvicenie1/CV1_funkcie.pdf         # zadanie 1 (+ Prepocitat!!!.png)
    ├── Cvicenie2/CV_gradientne.pdf       # zadanie 2 (+ fotky z tabule)
    ├── Cvicenie3/Zadanie3.pdf            # zadanie 3 (+ fotky z tabule, aj od Tomáša)
    ├── Cvicenie4/Zadanie4.pdf            # zadanie 4 (+ POZNAMKY!!!!.txt – zatiaľ prázdne)
    ├── optim_course.zip                  # pôvodný archív kostry
    └── optim_course/                     # ★ PRACOVNÁ KÓPIA – tu sa programuje
        └── optim_course/                 # nedotknutá kópia z archívu (originál s TODO) – NEUPRAVOVAŤ
```

> **Pozor:** upravuje sa vždy `Cvicenia/optim_course/<priečinok>/...`, nie vnorený
> `Cvicenia/optim_course/optim_course/...` (ten slúži len ako referencia originálu).

## Kostra `optim_course`

| Priečinok | Obsah |
|---|---|
| `scr/` | **spúšťacie skripty** jednotlivých cvičení (`*_scr.m`) – spúšťať z tohto priečinka, cesty pridávajú cez `addpath("..\xyz")` |
| `fcn/` | účelové funkcie vracajúce `[J,g,H]` (hodnota, gradient, Hessián): `fcn_QF`, `fcn_himmelblau`, `fcn_LF` |
| `optim_alg/` | optimalizačné algoritmy: gradientná, s optimálnym krokom, Newton-Raphson, Gauss-Newton, Nelder-Mead, linprog, quadprog, `optim_con` |
| `geometry/` | rotačné matice `rotX/Y/Z`, ich derivácie `drotX/Y/Z`, `RPY`, `RPY_diff`, `wrapToPi` |
| `ICP/` | ICP 2D/3D, odhad transformácie, mapa (`gen_map`), simulácia lidaru, `teapot.mat` |
| `robot_arm/` | kinematika manipulátora: `get_R_tot`, `get_dR_tot`, `get_Jacobian`, `transform_pi_to_p0`, `solve_IK`, `solve_IK_orientation`, `plot_robot` |
| `misc/` | pomocné: vykresľovanie (`plot_2D_function`, `plot_2D_optim`, …), `ode4step`, `ss_discretize`, kovariančné elipsy |
| `MPC/`, `drone/`, `rocket/`, `lab_models/`, `mobile_robot/`, `ident/` | modely a úlohy pre neskoršie cvičenia (MPC, LQ, Pontryagin, identifikácia) |
| `SLAM_KITTI/`, `SLAM_RPLIDAR/` | SLAM (MEX súbory `.mexw64`, C++ zdrojáky, Eigen, nanoflann) |

Skripty v `scr/`: `functions_scr`, `optim_scr`, `Kalman_scr`, `EKF_scr`, `lidar_scr`,
`robot_arm_scr`, `transform_fitting_scr`, `linprog_scr`, `simplex_scr`, `quadprog_scr`,
`Lagrange_scr`, `ident_scr`, `pendulum_scr`, `transmission_scr`, `discrete_LQ_scr`,
`MPC_scr`, `drone_scr_MPC`, `rocket_scr`, `mobile_robot_Pontryagin_scr`.

## Zadania

### Cvičenie 1 – Funkcie viacerých premenných (`CV1_funkcie.pdf`) ✅
- Skript: `scr/functions_scr.m`
- Kvadratická forma `J = ½xᵀAx + bᵀx + c`, `∇J = Ax + b`, `∇²J = A`, stacionárny bod `x* = -A\b`
- Rozklad na vlastné čísla `[Q,Lambda] = eig(A)` → definitnosť → min / max / sedlo
- Tri prípady A: (a) `[1 .5; .5 3]`, (b) `-[1.5 .2; .2 2]`, (c) `[-2 .5; .5 3]`; `b=[-6;10]`, `c=10`
- Himmelblau `J = (x1²+x2-11)² + (x1+x2²-7)²` – gradient a Hessián ručne
- Implementované: `fcn/fcn_QF.m`, `fcn/fcn_himmelblau.m`

### Cvičenie 2 – Gradientné metódy (`CV_gradientne.pdf`) ✅
- Skript: `scr/optim_scr.m`
- `optim_gradient` – `x = x - γ·g`, stop `gᵀg < ε`
- `optim_gradient_optimal_step` – `γ = gᵀg / (gᵀHg)` (kolmé po sebe idúce smery)
- `optim_newton_raphson` – `x = x - H\g`
- Analyzovať: vplyv γ, ε, počiatočného bodu, rýchlosť konvergencie, kolmosť smerov

### Cvičenie 3 – 2D lokalizácia robota lidarom, ICP + EKF (`Zadanie3.pdf`) ✅
- Skript: `scr/lidar_scr.m`
- Model: `x(k+1) = x + vΔt·cosθ`, `y(k+1) = y + vΔt·sinθ`, `θ(k+1) = θ + ωΔt`
- ICP: najbližší susedia `knnsearch(KDtree, pA_transformed,'k',1)`, odhad `R(θ)pA + t ≈ pB`
  nelineárnymi najmenšími štvorcami (Gauss-Newton s regularizáciou Λ)
- Gauss-Newton: `H = JᵀJ`, `g = Jᵀr`, `Δ = -(H+Λ)\g`
- EKF: `A = [1 0 -vΔt sinθ; 0 1 vΔt cosθ; 0 0 1]`, `C = I₃`, meranie = výstup ICP `[x;y;θ]`
- Implementované: `geometry/rotZ.m`, `drotZ.m`, `optim_alg/optim_GaussNewton.m`,
  `ICP/transform_estimate2D.m`, `ICP/ICP_estimate_2D.m`, EKF časť v `lidar_scr.m`

### Cvičenie 4 – Inverzná kinematika 6DOF manipulátora (`Zadanie4.pdf`) ✅
- Skript: `scr/robot_arm_scr.m`
- Kinematika: `p0(θ) = Rz(θ1)(T1 + Rx(θ2)(T2 + Rx(θ3)(T3 + Rz(θ4)(T4 + Ry(θ5)(T5 + Rz(θ6)T6)))))`
- `R = {@rotZ,@rotX,@rotX,@rotZ,@rotY,@rotZ}`, `dR = {@drotZ,@drotX,...}`, `Ti = [0;0;Li]`
- `R_T(θ) = Π Ri(θi)`; derivácia podľa θj = súčin, kde j-ta matica je nahradená `dRj`
- Rezíduá: `rp = p0(θ) - pr`; `ro` = prvky `Re = R_T(θ)·R_Rᵀ` na pozíciách (2,1), (3,1), (3,2)
- Newtonov krok: `(J + Λ)Δθ = -[rp; ro]`, `J = [Jp; Jo]` (6×6), stop `rpᵀrp + roᵀro < ε`
- Implementované: `geometry/rotX, rotY, drotX, drotY`, `robot_arm/get_R_tot`, `get_dR_tot`,
  `get_Jacobian` (derivácia vnoreného výrazu od vnútra von), `transform_pi_to_p0`
  (`p = R_j(θ_j)(T_j + p)` pre j = i..1), `solve_IK`, `solve_IK_orientation` – overené
  numerickým Jakobiánom, IK konverguje (chyba polohy ~1e-4)
- `robot_arm_scr`: sekcia „6 DOF Robot Initialization“ bola v pôvodnej kostre pred 3DOF IK
  (prepísala `R`, `T` → „Index exceeds…“). Presunutá za 3DOF IK, skript ide spustiť celý.
- Test: `pr = [1; 0.5; 2]`, `Rr = rotZ(π/3)·rotY(π/3)·rotX(π/3)`,
  `θ0 = [π;0;π/2;π;π/2;π/2]`, `Λ = 0.1·I`, `tol = 1e-6`

## Konvencie a tipy

- Funkcie cieľa vždy vracajú `[J, g, H]`; algoritmy volajú `J_fun(x_k)`.
- Vektory sú stĺpcové; mračná bodov `pA`, `pB` sú `N×2` (riadok = bod).
- Kostra niekedy ponechá komentár `%TODO` aj po doplnení – stav posudzovať podľa
  toho, či je za `=` reálny výraz (nie `%TODO` ani `[]`).
- Pri nových zadaniach: PDF dať do `Cvicenia/CvicenieN/`, riešenie písať do
  pracovnej kópie `optim_course/` a tento README aktualizovať (sekcia **Zadania**).
- Spustenie z príkazového riadka (príklad):
  `matlab -batch "cd('Cvicenia/optim_course/scr'); robot_arm_scr"`
