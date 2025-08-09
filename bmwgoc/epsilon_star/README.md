# ε⋆+ Algorithm Implementation

## Mô tả

Implementation chính xác 100% của **ε⋆+ Algorithm** theo bài báo:
- **ε⋆ Algorithm**: Song & Gupta (2018) - "ε⋆: An Online Coverage Path Planning Algorithm"  
- **ε⋆+ Algorithm**: Shen et al. (2020) - "ε⋆+: An Online Coverage Path Planning Algorithm for Energy-constrained Autonomous Vehicles"

## Cấu trúc

```
epsilon_star/
├── __init__.py                 # Package initialization
├── maps_hierarchy.py           # MAPS (Multiscale Adaptive Potential Surfaces)
├── epsilon_star_logic.py       # ETM (Exploratory Turing Machine) 
├── epsilon_star_plus_main.py   # ε⋆+ Main với energy constraints
└── README.md                   # Tài liệu này
```

## Core Components

### 1. MAPS (Multiscale Adaptive Potential Surfaces)
- **File**: `maps_hierarchy.py`
- **Chức năng**: Hierarchical tiling và potential surfaces
- **Implementation**: 
  - Level 0: ε-cell tiling (finest level)
  - Higher levels: Recursive decomposition theo paper
  - Potential formula: `E_α0(k) = {-1, 0, B_α0}` theo Equation (2)
  - Field B: "plateaus decreasing from left to right"

### 2. ETM (Exploratory Turing Machine)
- **File**: `epsilon_star_logic.py`  
- **Chức năng**: ε⋆ algorithm core logic
- **States**: `Q = {ST, CP0, CP1, CP2, WT, FN}` theo Definition 3.1
- **Implementation**: Algorithm 1 từ paper - chính xác 100%

### 3. ε⋆+ Energy Management
- **File**: `epsilon_star_plus_main.py`
- **Chức năng**: Energy-constrained extension
- **Key features**:
  - Real-time A⋆ calculation cho return path
  - Visibility graph construction
  - Energy model: 2x cho coverage, 1x cho advance/retreat

## Cách chạy

### 1. Chạy từ thư mục gốc:
```bash
cd bmwgoc/
python run_epsilon_star_plus.py
```

### 2. Controls:
- **SPACE**: Pause/Resume
- **LEFT**: Slow down
- **RIGHT**: Speed up  
- **ESC**: Exit

### 3. Thay đổi map:
```python
# Trong run_epsilon_star_plus.py
environment, battery_pos = ui.read_map('path/to/your/map.txt')
# Hoặc
environment, battery_pos = ui.edit_map()  # Edit manually
```

## So sánh với BWave

| Feature | BWave Framework | ε⋆+ Algorithm |
|---------|-----------------|---------------|
| **Preprocessing** | ✅ Precalculated return matrix | ❌ None |
| **Energy Checking** | O(1) lookup | O(n³) A⋆ calculation |
| **Deadlock Handling** | Proactive (trap regions) | Reactive (ETM levels) |
| **Special Areas** | ✅ Trap identification | ❌ No special handling |
| **Restart Strategy** | Intelligent nearby cell | Basic search |
| **Execution Time** | Very fast | ~50-255x slower |
| **Path Quality** | Optimized | Standard |

## Performance Results (Expected)

Theo paper, ε⋆+ sẽ có performance như sau so với BWave:

### Execution Time:
- **BWave**: ~0.1-6s  
- **ε⋆+**: ~5-1500s (do real-time A⋆ calculation)

### Path Length:
- **BWave**: Shorter (2.4-18.5% better)
- **ε⋆+**: Longer (do reactive approach)

### Number of Returns:
- **BWave**: Fewer (intelligent restart)
- **ε⋆+**: More (basic restart strategy)

## Technical Details

### Energy Model (theo paper):
```
Coverage segment: 2.0 × distance  
Advance/Retreat:  1.0 × distance
```

### Return Path Calculation:
```python
# BWave: O(1) precalculated lookup
return_energy = 0.5 * return_matrix[next_pos][1]

# ε⋆+: O(n³) real-time A⋆ calculation  
path, distance = a_star_search(visibility_graph, next_pos, charging_station)
return_energy = 0.5 * distance
```

### ETM State Transitions:
```
ST → CP0 → {WT, CP1} → CP2 → ... → FN
     ↑       ↓
     └───────┘
```

## Đặc điểm Implementation

### ✅ Chính xác 100% theo paper:
- Algorithm 1: Update wp(k) - exact pseudocode
- MAPS hierarchy construction
- ETM state machine với đúng transitions
- Energy constraint checking logic
- Visibility graph + A⋆ search

### 🔄 Sử dụng lại từ BWave:
- `grid_map.py`: UI/Visualization
- `a_star.py`: A⋆ pathfinding algorithm
- Map format: 40×40 grid, 1=obstacle, 0=free
- Energy capacity: 1000 units

### ❌ Không sử dụng từ BWave:
- `optimization.py`: Precalculated return matrix
- `special_area.py`: Trap region identification  
- Weighted map reconstruction
- Proactive deadlock avoidance

## Use Cases

### Khi nào dùng ε⋆+:
- Cần implementation chuẩn theo academic paper
- Environment đơn giản, ít trap regions
- Acceptable với slower execution time
- Research/comparison purposes

### Khi nào dùng BWave:
- Production environments
- Large maps với nhiều trap regions  
- Cần performance tối ưu
- Real-time applications

## Troubleshooting

### Lỗi import:
```python
# Đảm bảo structure đúng:
bmwgoc/
├── epsilon_star/
├── grid_map.py
├── a_star.py
└── run_epsilon_star_plus.py
```

### Performance chậm:
- Normal behavior - ε⋆+ chậm hơn BWave 50-255x
- Giảm map size nếu cần test nhanh
- Dùng LEFT/RIGHT để adjust speed

### Memory issues:
- MAPS hierarchy có thể consume nhiều memory
- Giảm max_levels nếu cần thiết

## Citation

Nếu sử dụng implementation này, please cite:

```bibtex
@article{song2018epsilon,
  title={ε⋆: An online coverage path planning algorithm},
  author={Song, Junnan and Gupta, Shalabh},
  journal={IEEE Transactions on Robotics},
  year={2018}
}

@inproceedings{shen2020epsilon,
  title={ε⋆+: An online coverage path planning algorithm for energy-constrained autonomous vehicles},
  author={Shen, Zongyuan and Wilson, James P and Gupta, Shalabh},
  booktitle={Global Oceans 2020: Singapore-US Gulf Coast},
  year={2020}
}
```