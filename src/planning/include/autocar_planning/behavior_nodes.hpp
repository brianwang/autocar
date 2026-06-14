#pragma once

#include <string>
#include <functional>
#include <memory>
#include <vector>

namespace autocar_planning {

enum class NodeStatus {
  SUCCESS,
  FAILURE,
  RUNNING
};

class BehaviorNode {
public:
  virtual ~BehaviorNode() = default;
  virtual NodeStatus tick() = 0;
  virtual void reset() {}
  const std::string& name() const { return name_; }

protected:
  std::string name_;
};

class Sequence : public BehaviorNode {
public:
  explicit Sequence(const std::string& name);
  void addChild(std::shared_ptr<BehaviorNode> child);
  NodeStatus tick() override;
  void reset() override;

private:
  std::vector<std::shared_ptr<BehaviorNode>> children_;
  size_t current_index_{0};
};

class Fallback : public BehaviorNode {
public:
  explicit Fallback(const std::string& name);
  void addChild(std::shared_ptr<BehaviorNode> child);
  NodeStatus tick() override;
  void reset() override;

private:
  std::vector<std::shared_ptr<BehaviorNode>> children_;
  size_t current_index_{0};
};

class ReactiveSequence : public BehaviorNode {
public:
  explicit ReactiveSequence(const std::string& name);
  void addChild(std::shared_ptr<BehaviorNode> child);
  NodeStatus tick() override;
  void reset() override;

private:
  std::vector<std::shared_ptr<BehaviorNode>> children_;
};

class ConditionNode : public BehaviorNode {
public:
  explicit ConditionNode(const std::string& name,
                          std::function<bool()> condition);
  NodeStatus tick() override;

private:
  std::function<bool()> condition_;
};

class ActionNode : public BehaviorNode {
public:
  explicit ActionNode(const std::string& name,
                       std::function<NodeStatus()> action);
  NodeStatus tick() override;

private:
  std::function<NodeStatus()> action_;
};

}  // namespace autocar_planning
