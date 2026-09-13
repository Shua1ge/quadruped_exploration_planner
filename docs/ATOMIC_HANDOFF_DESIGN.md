# 区域承诺原子接管设计方案

## 问题定义

当前 `activate_prepared_observation()` 在第 632-641 行硬性阻止跨区域切换：
```python
if (self.active_region_id is not None
        and candidate.region_id != self.active_region_id
        and self.commitment_state == "COMMITTED"):
    return False  # 直接拒绝
```

这导致即使当前任务已完成、prepared path 已验证安全，也无法原子切换到新区域。

---

## 设计方案

### 核心原则

**区域承诺只保护"仍在执行、仍有探索价值"的任务，不能在任务完成后继续阻止切换。**

### 状态转换图

```
┌─────────────────────────────────────────────────────┐
│ COMMITTED (执行中)                                   │
│ - active_region_id = R1                             │
│ - active_observation.progress < observation_done    │
└────────────────┬────────────────────────────────────┘
                 │
                 │ observation_done 或 REACHED
                 ↓
┌─────────────────────────────────────────────────────┐
│ HANDOFF_ELIGIBLE (可交接)                           │
│ - 当前任务已完成，但承诺未释放                        │
│ - prepared_candidate 可以跨区域验证                  │
└────────────────┬────────────────────────────────────┘
                 │
                 │ 验证 prepared_path (Dense A*)
                 ↓
        ┌────────┴────────┐
        │                 │
   验证成功             验证失败
        │                 │
        ↓                 ↓
  ┌─────────┐      ┌──────────────┐
  │原子切换  │      │保留旧状态     │
  │- 发布新路径│    │- 等待地图更新│
  │- 承诺转移│      │- 或重新规划   │
  └─────────┘      └──────────────┘
```

---

## 实现方案

### 修改 1：引入 HANDOFF_ELIGIBLE 状态

在 `FrontierExplorer.__init__()` 中：
```python
self.commitment_state = "UNCOMMITTED"
# 可能的值：
# - "UNCOMMITTED": 无承诺
# - "COMMITTED": 执行中，保护当前区域
# - "HANDOFF_ELIGIBLE": 任务已完成，可跨区域交接
# - "RELEASABLE": 承诺已释放，可重新选择
```

### 修改 2：在任务完成时进入 HANDOFF_ELIGIBLE

在 `handoff_satisfied_observation()` 中：
```python
def handoff_satisfied_observation(self, status: str) -> bool:
    """原子切换：先验证 prepared path，成功后原子转移承诺。"""
    if self.active_goal is None:
        return False
    if self.last_handoff_attempt_update == self.map_update_count:
        return False
    self.last_handoff_attempt_update = self.map_update_count
    previous_goal = self.active_goal
    previous_region = self.active_region_id
    
    # 关键：进入 HANDOFF_ELIGIBLE，允许跨区域验证
    old_commitment_state = self.commitment_state
    self.commitment_state = "HANDOFF_ELIGIBLE"
    
    # 尝试激活 prepared_candidate（现在可以跨区域）
    switched = self.activate_prepared_observation()
    
    if switched:
        # 成功切换，承诺已在 activate_candidate() 中转移
        if not self.is_blacklisted(previous_goal):
            self.completed_goals.append(previous_goal)
        self.observations_satisfied += 1
        self.publish_status(status)
        self.get_logger().info(
            f"[ATOMIC_HANDOFF_SUCCESS] from_region={previous_region} "
            f"to_region={self.active_region_id}")
        return True
    
    # prepared path 不可用，尝试完整规划
    switched = self.plan_from_current_position(excluded_goals=(previous_goal,))
    
    if switched:
        if not self.is_blacklisted(previous_goal):
            self.completed_goals.append(previous_goal)
        self.observations_satisfied += 1
        self.publish_status(status)
        return True
    
    # 都失败了，恢复旧状态
    self.commitment_state = old_commitment_state
    
    if self.latest_frontier_count == 0:
        self.finish_active_observation("EXPLORATION_COMPLETE")
        return True
    
    # 保留旧任务，等待安全切换
    self.publish_status("WAITING_FOR_SAFE_HANDOFF")
    return False
```

### 修改 3：放宽 activate_prepared_observation() 的跨区域限制

在 `activate_prepared_observation()` 中：
```python
def activate_prepared_observation(self) -> bool:
    """Reuse the safe suffix of a prepared route from the current pose."""
    candidate = self.prepared_candidate
    if self.position is None or candidate is None:
        return False
    
    # 修改：只在 COMMITTED 且任务未完成时才阻止跨区域
    if (self.active_region_id is not None
            and candidate.region_id != self.active_region_id
            and self.commitment_state == "COMMITTED"):  # HANDOFF_ELIGIBLE 时允许
        self.get_logger().info(
            f"[PREPARED_HANDOFF_DEFERRED] prepared_region="
            f"{candidate.region_id} committed_region={self.active_region_id} "
            f"commitment_state={self.commitment_state}",
            throttle_duration_sec=2.0)
        return False
    
    # ... 后续 Dense 验证逻辑保持不变 ...
    # 验证成功后，activate_candidate() 会自动转移承诺
```

---

## 改进 2：区域级 Dense 失败反馈

### 问题

当前 `materialize_sparse_candidate()` 只标记单个 goal 冷却：
```python
self.add_goal_failure_cooldown(
    candidate.goal, "DENSE_FINAL_VALIDATION_FAILED")
```

同一区域的其他 viewpoint 继续尝试，导致验证风暴。

### 方案

**引入区域级 Dense 失败记录：**

```python
class FrontierExplorer:
    def __init__(self):
        # 新增：区域的 Dense 不可达记录
        self.region_dense_unreachable: Dict[int, int] = {}
        # key: region_id
        # value: map_content_revision
```

**在 Dense 验证失败时标记区域：**

```python
def materialize_sparse_candidate(
        self, start: Cell, candidate: FrontierCandidate,
        inflated: Set[Cell], blocked_edges: Set[DirectedEdge]
        ) -> Optional[FrontierCandidate]:
    """Recover and validate only the sparse-selected route on the dense map."""
    self.dense_final_validation_searches += 1
    path = astar_known(
        self.grid, start, candidate.cell, inflated, blocked_edges)
    
    if not path:
        self.sparse_final_validation_failures += 1
        self.add_goal_failure_cooldown(
            candidate.goal, "DENSE_FINAL_VALIDATION_FAILED")
        
        # 新增：标记区域在当前 revision 不可达
        self.region_dense_unreachable[candidate.region_id] = (
            self.map_content_revision)
        
        self.get_logger().warning(
            f"[SPARSE_ROUTE_REJECTED] region={candidate.region_id} "
            f"map_revision={self.map_content_revision} "
            f"goal=({candidate.goal[0]:.2f},{candidate.goal[1]:.2f}); "
            "dense final validation found no route")
        return None
    
    # ... 成功逻辑 ...
```

**在区域选择时过滤 Dense 不可达区域：**

```python
def choose_hierarchical_candidate(
        self, start: Cell, regions: Sequence[FrontierRegion],
        candidates: Sequence[FrontierCandidate], ...):
    # ... 前置逻辑 ...
    
    # 过滤当前 revision 已知 Dense 不可达的区域
    available_candidates = [
        c for c in candidates
        if self.region_dense_unreachable.get(c.region_id, -1)
           != self.map_content_revision
    ]
    
    if not available_candidates:
        # 所有区域都标记为不可达，清空记录重试
        self.region_dense_unreachable.clear()
        self.get_logger().warning(
            "[ALL_REGIONS_DENSE_UNREACHABLE] clearing cache and retrying")
        available_candidates = candidates
    
    # ... 后续选择逻辑使用 available_candidates ...
```

**在地图更新时清理过期记录：**

```python
def cloud_callback(self, msg: PointCloud2):
    # ... 地图融合逻辑 ...
    
    self.map_content_revision += 1
    
    # 清理过期的 Dense 不可达记录
    # 只保留当前 revision 和上一个 revision 的记录
    self.region_dense_unreachable = {
        region_id: revision
        for region_id, revision in self.region_dense_unreachable.items()
        if revision >= self.map_content_revision - 1
    }
```

---

## 改进 3：ATSP 计算触发优化

### 问题

当前可能因为 frontier centroid 小幅漂移，每次都重算 ATSP。

### 方案

**引入 ATSP 缓存键：**

```python
class FrontierExplorer:
    def __init__(self):
        self.last_atsp_cache_key: Optional[Tuple] = None
        self.last_atsp_result: List[int] = []
```

**生成稳定的缓存键：**

```python
def compute_atsp_cache_key(
        self, regions: Sequence[FrontierRegion],
        sparse_graph_revision: int) -> Tuple:
    """生成稳定的 ATSP 缓存键，忽略小幅 centroid 漂移"""
    region_ids = tuple(sorted(r.region_id for r in regions))
    region_sizes = tuple(len(r.cells) for r in regions)
    return (
        region_ids,
        region_sizes,
        sparse_graph_revision,
        self.route_constraint_revision
    )
```

**在 ATSP 计算前检查缓存：**

```python
def solve_region_sequence(
        self, regions: Sequence[FrontierRegion], ...):
    # 生成缓存键
    cache_key = self.compute_atsp_cache_key(
        regions, self.sparse_router.graph_revision)
    
    # 缓存命中，直接返回
    if cache_key == self.last_atsp_cache_key:
        self.get_logger().debug(
            "[ATSP_CACHE_HIT] reusing previous sequence")
        return self.last_atsp_result
    
    # 缓存未命中，重新计算
    self.get_logger().info(
        f"[ATSP_RECOMPUTE] regions={len(regions)} "
        f"prev_key={self.last_atsp_cache_key is not None}")
    
    sequence = solve_open_held_karp(...)  # 实际 ATSP 求解
    
    # 更新缓存
    self.last_atsp_cache_key = cache_key
    self.last_atsp_result = sequence
    return sequence
```

---

## 实施优先级

### 阶段 1：原子接管（立即）
1. 引入 `HANDOFF_ELIGIBLE` 状态
2. 修改 `handoff_satisfied_observation()` 实现原子切换
3. 放宽 `activate_prepared_observation()` 的跨区域限制
4. **验证**：日志中应看到 `[ATOMIC_HANDOFF_SUCCESS]`，无 `WAITING_FOR_SAFE_HANDOFF`

### 阶段 2：Dense 失败反馈（同步）
1. 添加 `region_dense_unreachable` 字典
2. 在 `materialize_sparse_candidate()` 失败时标记区域
3. 在候选选择时过滤标记区域
4. **验证**：573-590s 类似的验证风暴应消失，日志显示 `[ALL_REGIONS_DENSE_UNREACHABLE]` 次数减少

### 阶段 3：ATSP 缓存（后续）
1. 实现 `compute_atsp_cache_key()`
2. 在 ATSP 求解前检查缓存
3. **验证**：日志显示 `[ATSP_CACHE_HIT]` 比例，`region_sequence_ms` 降低

---

## 测试验证

### 关键日志指标

```bash
# 原子接管成功率
grep "ATOMIC_HANDOFF_SUCCESS\|WAITING_FOR_SAFE_HANDOFF" log.txt

# Dense 失败风暴
grep "SPARSE_ROUTE_REJECTED" log.txt | wc -l

# ATSP 缓存命中
grep "ATSP_CACHE_HIT\|ATSP_RECOMPUTE" log.txt
```

### 预期改进

- **原子接管**：`WAITING_FOR_SAFE_HANDOFF` 消失
- **Dense 反馈**：连续 `SPARSE_ROUTE_REJECTED` 从几十次 → 1-2 次
- **ATSP 缓存**：`region_sequence_ms` 从 1-1.5s → <100ms（缓存命中时）

---

## 风险与权衡

### 风险 1：HANDOFF_ELIGIBLE 窗口的安全性
- **缓解**：Dense 验证仍然执行，不安全的 prepared path 会被拒绝

### 风险 2：区域级标记过于激进
- **缓解**：只保留当前和前一个 revision，地图变化后自动重试

### 风险 3：ATSP 缓存键稳定性
- **缓解**：使用 region_id 和 size，而非 centroid 坐标
