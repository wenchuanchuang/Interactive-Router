Only the project structure and architecture have been set up so far. Implementation will follow.

Some code in this project is AI-assisted.

This is an **interactive rip-up and reroute assistant tool** designed for KiCad (`.kicad_pcb`).

## System Architecture and Data Flow

1. **[Python] Parsing and Rip-up:**
   Uses KiCad’s native `pcbnew` API to read the design file and perform user-triggered localized net rip-up.

2. **[Python → C++] Task Dispatching:**
   The remaining Pads, Vias, and Tracks are treated as “obstacles”, and together with the routing tasks, are passed through `pybind11`.

3. **[C++] Rasterization and Routing:**
   A 3D grid is constructed on the C++ side, where obstacle expansion and rasterization are performed, followed by execution of the routing algorithm.

4. **[C++ → Python] Rendering:**
   The computed path nodes are returned to Python as a `NumPy Array`, rendered in 2D using PyQt, and written back into the `.kicad_pcb` file.

## Project Structure

```text
This project/
├── CMakeLists.txt         # Build script for C++ and pybind11
├── requirements.txt       # Python dependency list
│
├── cpp_core/              # C++ routing core
│   ├── include/           
│   ├── src/               
│   └── pybind_api.cpp    
│
└── router_app/            # Python main application and interface
    ├── main.py            # Application entry point
    ├── kicad_parser.py    # pcbnew API wrapper and geometry data extraction
    └── gui/               # PyQt interactive interface implementation
```

---

這是一個針對 KiCad (`.kicad_pcb`) 設計的互動式拆線與重新繞線 輔助工具。

## 系統架構與資料流

1. **[Python] 解析與拆線:** 透過 KiCad `pcbnew` API 讀取設計檔，執行使用者觸發的局部網路拆線。
2. **[Python -> C++] 任務派發:** 將保留下來的 Pad、Via、Track 視為障礙物，連同 待繞線任務 透過 `pybind11` 傳遞。
3. **[C++] 光柵化與尋路:** 在 C++ 端建立3D 網格，執行障礙物膨脹與光柵化，隨後啟動佈線演算法。
4. **[C++ -> Python] 渲染:** 將計算完畢的路徑節點以 `NumPy Array` 回傳至 Python，利用 PyQt 進行 2D 渲染，並寫回 `.kicad_pcb`。

##  Project Structure

```text
這份專案/
├── CMakeLists.txt         # C++ 與 pybind11 的編譯腳本
├── requirements.txt       # Python 相依套件清單
│
├── cpp_core/              #  C++ 路由核心
│   ├── include/           
│   ├── src/               
│   └── pybind_api.cpp    
│
└── router_app/            # Python 主程式與介面
    ├── main.py            # 應用程式進入點
    ├── kicad_parser.py    # pcbnew API 封裝與幾何資料提取
    └── gui/               # PyQt 互動介面實作
```
