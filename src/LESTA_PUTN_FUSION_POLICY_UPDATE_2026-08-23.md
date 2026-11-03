# LeSTA-PUTN 融合策略修正（2026-08-23）

## 修改边界

- 不修改 `src/LeSTA` 下的网络、特征、推理、概率更新或建图实现。
- 只修改 PUTN 侧如何读取和融合 LeSTA 输出，以及自动化评测配置和可视化。
- `use_lesta:=false` 时仍是 PUTN 原算法，不受本次融合策略影响。

## 问题依据

会话 6 侧翻位置附近，LeSTA 累积层 `mapping/probability` 的中位数和上限均为
`0.95`，导致该区域在 RViz 中几乎全绿。旧加权融合会把 PUTN 已识别的高风险降低：

```text
PUTN risk = 0.8648
LeSTA probability = 0.95, LeSTA risk = 0.05
legacy = 0.75 * 0.8648 + 0.25 * 0.05 = 0.6611
```

由于 `0.6611 < 0.70`，旧策略可能放行 PUTN 几何风险已经超阈值的位置。

## 新策略

自动化融合测试默认采用：

```text
weighted = putn_weight * putn_risk + lesta_weight * lesta_risk
fused_risk = max(putn_risk, weighted)
```

策略名为 `risk_floor_weighted`。它具有两个约束：

1. LeSTA 可以提高风险，参与危险区域判定。
2. LeSTA 不能降低 PUTN 几何风险，避免域外高置信度将危险地形错误“洗绿”。

另外保留两个可选策略：

- `legacy_weighted_sum`：旧加权融合，用于历史结果复现。
- `conservative_max`：`max(putn_risk, lesta_risk)`，用于更保守的消融实验。

## 输入层调整

规划默认仍订阅 LeSTA mapping 的全地图消息，但融合读取：

- `traversability/probability`
- `traversability/binary`

即采用当前帧推理结果，避开累积 `mapping/probability` 长时间饱和到 `0.95` 的影响。
累积层仍保留作诊断显示，不参与默认融合。

## 推荐首轮验证

先进行一次 C 到 A 的独立短测试，不录 bag；确认能够完成规划、车辆没有明显偏离或侧翻后，
再启动正式多样本实验。新实验必须使用新的 `method` 和 `campaign_id`，不能续接旧的
`mapping + legacy_weighted_sum` 检查点。

关键参数应显式固定为：

```text
lesta_fusion_policy:=risk_floor_weighted
lesta_probability_layer:=traversability/probability
lesta_binary_layer:=traversability/binary
lesta_putn_weight:=0.75
lesta_weight:=0.25
lesta_risk_threshold:=0.70
```
