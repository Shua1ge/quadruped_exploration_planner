# Explorer 优化提案

> **状态（2026-09-13）：backlog。四项优化截至本文更新日均未实现**
> （已核实代码中无 `observation_cache` / standoff 环预计算 / 四叉树 / 自适应边采样）。
> 实施前先按 §测量基准取一轮 `[GLOBAL_PLAN_BASELINE]` 实测数据，
> 确认热点确实在对应环节再动手。代码位置可能与文中行号有出入，以当前代码为准。

## 背景

当前 Explorer 已经实现了拓扑感知区域划分和稀疏路由，但仍有几个可以进一步优化的计算热点。

## 优化 1：观测目标单元的缓存与增量更新

### 当前实现

`build_frontier_candidates()` 中，每个 frontier 都调用：
```python
target_cells = observation_target_cells(
    self.grid, frontier, self.observation_radius)
```

这会遍历半径 3m 的圆形区域（约 450 个栅格，0.2m 分辨率），检查哪些是 `UNKNOWN`。

### 问题

1. **重复计算**：相邻 frontier 的观测范围大量重叠，但每次都独立计算
2. **无缓存**：即使地图没有更新，每次规划周期（0.25s）都重新计算
3. **热点**：如果有 20 个 frontier × 5 个候选点 = 100 次圆形遍历

### 优化方案

#### 方案 A：增量地图更新触发缓存失效

```python
class FrontierExplorer:
    def __init__(self):
        self.observation_cache: Dict[Cell, Tuple[Set[Cell], int]] = {}
        # key: frontier cell
        # value: (target_cells, map_revision)
    
    def cloud_callback(self, msg):
        # ... 地图融合 ...
        self.map_content_revision += 1
        
        # 清理受影响区域的缓存
        changed_cells = self.grid.get_recently_changed_cells()
        for frontier, (cached_targets, revision) in list(self.observation_cache.items()):
            if any(cell in changed_cells for cell in cached_targets):
                del self.observation_cache[frontier]
    
    def get_observation_targets(self, frontier: Cell) -> Set[Cell]:
        cached = self.observation_cache.get(frontier)
        if cached is not None:
            targets, revision = cached
            if revision == self.map_content_revision:
                return targets
        
        # 缓存未命中，重新计算
        targets = observation_target_cells(
            self.grid, frontier, self.observation_radius)
        self.observation_cache[frontier] = (targets, self.map_content_revision)
        return targets
```

**预期收益**：
- 稳定区域：缓存命中率 70-90%
- 计算减少：遍历次数降低 70-80%

---

## 优化 2：Viewpoint 搜索的空间分区

### 当前实现

`safe_viewpoint_cells()` 在每个 frontier 附近搜索一个正方形区域：
```python
for dx in range(-search_cells, search_cells + 1):
    for dy in range(-search_cells, search_cells + 1):
        # 检查距离、自由空间、视线
```

search_cells ≈ 5-10，所以每次搜索约 100-400 个格子。

### 问题

1. 暴力遍历整个正方形
2. 大部分格子因距离不符合直接跳过
3. 每个 frontier 都独立搜索

### 优化方案

#### 方案 B：预先生成候选环

```python
def precompute_standoff_ring(stand_off: float, resolution: float) -> List[Tuple[int, int]]:
    """预计算符合 standoff 距离的相对偏移量"""
    desired_cells = stand_off / resolution
    tolerance_cells = 0.4 / resolution
    minimum_cells = desired_cells - tolerance_cells
    maximum_cells = desired_cells + tolerance_cells
    
    ring = []
    search_radius = int(math.ceil(maximum_cells))
    for dx in range(-search_radius, search_radius + 1):
        for dy in range(-search_radius, search_radius + 1):
            distance = math.hypot(dx, dy)
            if minimum_cells <= distance <= maximum_cells:
                ring.append((dx, dy))
    return ring

# 初始化时计算一次
STANDOFF_RING = precompute_standoff_ring(1.0, 0.2)  # 全局常量

def safe_viewpoint_cells_optimized(grid, frontier, inflated, inward):
    """使用预计算的环形偏移"""
    ranked = []
    for dx, dy in STANDOFF_RING:  # 只遍历环上的点
        cell = (frontier[0] + dx, frontier[1] + dy)
        if not grid.planning_free(cell, inflated):
            continue
        # ... 其他检查 ...
```

**预期收益**：
- 遍历点数：100-400 → 30-50（减少 80%）
- 适用于所有 frontier，计算一次

---

## 优化 3：八叉树替换 Explorer 的二维栅格

### 当前实现

Explorer 使用稠密的二维占据网格：
- 分辨率：0.2m
- 尺寸：64m × 40m = 2560 m²
- 栅格数：2560 / 0.04 = 64,000 个

### 问题

虽然是二维，但仍然在每次融合时更新大量栅格。

### 优化方案

#### 方案 C：四叉树（2D Octree）

```python
class QuadTreeGrid:
    """稀疏的二维占据网格"""
    def __init__(self, size_x, size_y, min_resolution):
        self.root = QuadNode(0, 0, size_x, size_y)
        self.min_resolution = min_resolution
    
    def update_cell(self, x, y, value):
        """更新单个格子，自动合并/分裂节点"""
        self.root.set_value(x, y, value, self.min_resolution)
    
    def value(self, cell):
        return self.root.query(cell[0], cell[1])

class QuadNode:
    def __init__(self, x, y, width, height):
        self.bounds = (x, y, width, height)
        self.value = UNKNOWN
        self.children = None  # None = 叶节点
    
    def set_value(self, x, y, new_value, min_res):
        if self.is_leaf():
            if self.can_split(min_res):
                self.split()
                self.propagate_to_children(x, y, new_value, min_res)
            else:
                self.value = new_value
        else:
            quad = self.find_quad(x, y)
            self.children[quad].set_value(x, y, new_value, min_res)
            self.try_merge()
```

**预期收益**：
- 大片已探索空旷区域：自动合并为大节点
- 内存：64,000 格 → 估计 10,000-20,000 节点（减少 70%）
- 融合速度：只更新受影响的节点

**权衡**：
- 查询速度略慢（O(log n) vs O(1)）
- 但 Explorer 不是实时循环，0.25s 周期足够

---

## 优化 4：稀疏路由覆盖率提升

### 当前状态监控

日志中会输出：
```
sparse_candidate_hits=X sparse_candidate_fallbacks=Y
```

### 诊断

如果 `fallbacks / (hits + fallbacks) > 0.3`（未命中率 > 30%），说明拓扑覆盖不够。

### 优化方案

#### 方案 D：动态调整附着参数

```python
# launch 文件中
"sparse_attachment_radius": 1.5  # 当前

# 如果未命中率高，增大到：
"sparse_attachment_radius": 2.0  # 或更大
```

#### 方案 E：边采样密度自适应

在 `sparse_routing.py` 中：
```python
def sample_edge_adaptively(edge: SparseEdge):
    """根据边的曲率和长度调整采样密度"""
    if edge.length < 2.0:
        interval = 0.3  # 短边密集采样
    elif edge.minimum_clearance < 1.0:
        interval = 0.4  # 窄走廊密集采样
    else:
        interval = 0.6  # 宽阔区域稀疏采样
    
    return sample_polyline(edge.polyline, interval)
```

---

## 实施优先级

### 🥇 立即可做（高收益 + 低风险）

1. **Viewpoint 搜索预计算**（优化 2）
   - 全局常量，初始化一次
   - 减少 80% 的遍历点
   - 无需改接口

2. **观测目标缓存**（优化 1，简化版）
   - 先实现简单的 LRU 缓存
   - 基于 map_revision 失效
   - 预期 50-70% 命中率

### 🥈 中期改进（需要测试验证）

3. **稀疏路由调优**（优化 4）
   - 先从日志看实际未命中率
   - 如果 > 30%，调整 attachment_radius
   - 或实现自适应边采样

### 🥉 长期重构（需要架构变动）

4. **四叉树替换**（优化 3）
   - 需要完整测试
   - 保持接口兼容性
   - 可以作为独立实验分支

---

## 测量基准

在实施前后，记录这些指标：

```python
# 从日志提取
grep "GLOBAL_PLAN_BASELINE" log.txt | awk '{
    print $4, $6, $8, $10, $12
}' > baseline_before.csv

# 优化后对比
# - map_preprocess_ms
# - candidate_tree_ms
# - sparse_candidate_hits/fallbacks
# - total_ms
```

---

## 代码位置

- **优化 1**：`frontier_explorer.py` 的 `build_frontier_candidates()`
- **优化 2**：`explorer_core/frontier_regions.py` 的 `safe_viewpoint_cells()`
- **优化 3**：新建 `explorer_core/quadtree_grid.py`
- **优化 4**：`sparse_routing.py` 和 launch 参数
