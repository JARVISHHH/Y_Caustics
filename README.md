

### TPS Usage Example

```
std::vector<Eigen::Vector2f> constraints;
constraints.push_back(Eigen::Vector2f(0.4,0.5));
constraints.push_back(Eigen::Vector2f(0.2,0.6));

tps m_tps;
m_tps.init(constraints);
m_tps.tps_solve(Eigen::Vector2f(0.2,0.3));
```
