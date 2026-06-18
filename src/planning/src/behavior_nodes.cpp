#include "autocar_planning/behavior_nodes.hpp"

namespace autocar_planning {

Sequence::Sequence(const std::string& name) { name_ = name; }
void Sequence::addChild(std::shared_ptr<BehaviorNode> child) { children_.push_back(child); }

NodeStatus Sequence::tick() {
  for (; current_index_ < children_.size(); ++current_index_) {
    auto status = children_[current_index_]->tick();
    if (status != NodeStatus::SUCCESS) return status;
  }
  return NodeStatus::SUCCESS;
}

void Sequence::reset() {
  current_index_ = 0;
  for (auto& c : children_) c->reset();
}

Fallback::Fallback(const std::string& name) { name_ = name; }
void Fallback::addChild(std::shared_ptr<BehaviorNode> child) { children_.push_back(child); }

NodeStatus Fallback::tick() {
  for (; current_index_ < children_.size(); ++current_index_) {
    auto status = children_[current_index_]->tick();
    if (status != NodeStatus::FAILURE) return status;
  }
  return NodeStatus::FAILURE;
}

void Fallback::reset() {
  current_index_ = 0;
  for (auto& c : children_) c->reset();
}

ReactiveSequence::ReactiveSequence(const std::string& name) { name_ = name; }
void ReactiveSequence::addChild(std::shared_ptr<BehaviorNode> child) { children_.push_back(child); }

NodeStatus ReactiveSequence::tick() {
  for (auto& c : children_) {
    auto status = c->tick();
    if (status != NodeStatus::SUCCESS) return status;
  }
  return NodeStatus::SUCCESS;
}

void ReactiveSequence::reset() {
  for (auto& c : children_) c->reset();
}

ConditionNode::ConditionNode(const std::string& name, std::function<bool()> condition)
  : condition_(condition) { name_ = name; }

NodeStatus ConditionNode::tick() {
  return condition_() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

ActionNode::ActionNode(const std::string& name, std::function<NodeStatus()> action)
  : action_(action) { name_ = name; }

NodeStatus ActionNode::tick() {
  return action_();
}

}  // namespace autocar_planning
