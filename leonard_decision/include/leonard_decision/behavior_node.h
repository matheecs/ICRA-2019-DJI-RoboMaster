#ifndef MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H
#define MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H

#include <chrono>
#include <thread>
#include <vector>

#include "blackboard.h"
/*
                             BehaviorNode(Run, Reset, SetParent)
            /                            |                                         \
        ActionNode                 DecoratorNode                              CompositeNode
只是改了BehaviorType         BehaviorNode::Ptr child_node_ptr_               vector<BehaviorNode::Ptr> children_node_ptr_
                                         |                                    unsigned int children_node_index_
                                         |                                       /       |        \
                                  preconditionNode                        selecor     sequence     parallel
                                  virtual bool Reevaluation();
                                  std::function<bool()> precondition_function_;
                                  AbortType abort_type_;

 */


namespace decision {

    enum class BehaviorType {
        PARALLEL,
        SELECTOR,
        SEQUENCE,

        ACTION,

        PRECONDITION,
    };
    enum class BehaviorState {
        RUNNING,
        SUCCESS,
        FAILURE,
        IDLE,
    };
    enum class AbortType {
        NONE,//不终止执行
        SELF,//中止self，以及在此节点下运行的所有子树
        LOW_PRIORITY,//中止此节点右方的所有节点
        BOTH//中止self，以及在此节点下运行的所有子树，此节点右方的所有节点
    };

    class BehaviorNode : public std::enable_shared_from_this<BehaviorNode> {
    public:

        typedef std::shared_ptr<BehaviorNode> Ptr;

        BehaviorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr) :
                name_(name),
                behavior_type_(behavior_type),
                blackboard_ptr_(blackboard_ptr),
                behavior_state_(BehaviorState::IDLE) {}

        virtual ~BehaviorNode() = default;

        BehaviorState Run() {
            //如果没在run，初始化、update；还没run，OnTerminate
            if (behavior_state_ != BehaviorState::RUNNING) {
                OnInitialize();
            }

            behavior_state_ = Update();

            if (behavior_state_ != BehaviorState::RUNNING) {
                OnTerminate(behavior_state_);
            }

            return behavior_state_;
        }

        virtual void Reset() {
            if (behavior_state_ == BehaviorState::RUNNING) {
                behavior_state_ = BehaviorState::IDLE;
                OnTerminate(behavior_state_);
            }
        }

        BehaviorType GetBehaviorType() {
            return behavior_type_;
        }

        BehaviorState GetBehaviorState() {
            return behavior_state_;
        }

        std::string GetName() {
            return name_;
        }

        void SetParent(BehaviorNode::Ptr parent_node_ptr) {
            parent_node_ptr_ = parent_node_ptr;
        }

    protected:

        virtual BehaviorState Update() = 0;

        virtual void OnInitialize() = 0;

        virtual void OnTerminate(BehaviorState state) = 0;

        //! Node name
        std::string name_;
        //! State
//  std::mutex behavior_state_mutex_;
        BehaviorState behavior_state_;
        //! Type
        BehaviorType behavior_type_;
        //! Blackboard
        Blackboard::Ptr blackboard_ptr_;
        //! Parent Node Pointer
        BehaviorNode::Ptr parent_node_ptr_;
    };

    class ActionNode : public BehaviorNode {
    public:
        ActionNode(std::string name, const Blackboard::Ptr &blackboard_ptr) :
                BehaviorNode::BehaviorNode(name, BehaviorType::ACTION, blackboard_ptr) {}

        virtual ~ActionNode() = default;

    protected:
        virtual void OnInitialize() = 0;

        virtual BehaviorState Update() = 0;

        virtual void OnTerminate(BehaviorState state) = 0;

    };

    class DecoratorNode : public BehaviorNode {
    public:
        DecoratorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr,
                      const BehaviorNode::Ptr &child_node_ptr = nullptr) :
                BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
                child_node_ptr_(child_node_ptr) {}

        virtual ~DecoratorNode() = default;

        void SetChild(const BehaviorNode::Ptr &child_node_ptr) {
            child_node_ptr_ = child_node_ptr;
            child_node_ptr->SetParent(shared_from_this());
        }

    protected:
        virtual void OnInitialize() = 0;

        virtual BehaviorState Update() = 0;

        virtual void OnTerminate(BehaviorState state) = 0;

        BehaviorNode::Ptr child_node_ptr_;
    };

    class PreconditionNode : public DecoratorNode {
    public:
        PreconditionNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                         const BehaviorNode::Ptr &child_node_ptr = nullptr,
                         std::function<bool()> precondition_function = std::function<bool()>(),
                         AbortType abort_type = AbortType::NONE) :
                DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr, child_node_ptr),
                precondition_function_(precondition_function), abort_type_(abort_type) {}

        virtual ~PreconditionNode() = default;

        AbortType GetAbortType() {
            return abort_type_;
        }

    protected:

        virtual void OnInitialize() {
            //LOG_INFO << name_ << " " << __FUNCTION__;
        }

        virtual bool Precondition() {
            if (precondition_function_) {
                return precondition_function_();
            } else {
                //LOG_ERROR << "There is no chosen precondition function, then return false by default!";
                return false;
            }
        };

        virtual BehaviorState Update() {
            if (child_node_ptr_ == nullptr) {
                return BehaviorState::SUCCESS;
            }
            // Reevaluation
            if (Reevaluation()) {
                BehaviorState state = child_node_ptr_->Run();
                return state;
            }
            return BehaviorState::FAILURE;
        }

        virtual void OnTerminate(BehaviorState state) {
            // 在precondition中：不在running，会重置子节点，另外对precondition使用reset也会重置子节点
            // Reset会重置正在运行的子节点，方式是使变成BehaviorState::IDLE，再OnTerminate
            switch (state) {
                case BehaviorState::IDLE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " IDLE!";
                    //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
                    child_node_ptr_->Reset();
                    break;
                case BehaviorState::SUCCESS:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " SUCCESS!";
                    break;
                case BehaviorState::FAILURE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " FAILURE!";
                    //TODO: the following recovery measure is in failure situation caused by precondition false.
                    child_node_ptr_->Reset();
                    break;
                default:
                    //LOG_ERROR << name_ << " " << __FUNCTION__ << " ERROR!";
                    return;
            }
        }

        virtual bool Reevaluation();

        std::function<bool()> precondition_function_;
        AbortType abort_type_;
    };


    class MyWithoutRfidCondition : public PreconditionNode {
    public:
        MyWithoutRfidCondition(std::string name, const Blackboard::Ptr &blackboard_ptr,
                               const BehaviorNode::Ptr &child_node_ptr = nullptr,
                               std::function<bool()> precondition_function = std::function<bool()>(),
                               AbortType abort_type = AbortType::NONE) : PreconditionNode::PreconditionNode(name,
                                                                                                            blackboard_ptr,
                                                                                                            child_node_ptr,
                                                                                                            precondition_function,
                                                                                                            abort_type) {}

        virtual ~MyWithoutRfidCondition() = default;

    protected:

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    child_node_ptr_->Reset();
                    break;
                case BehaviorState::SUCCESS:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " SUCCESS!";
                    break;
                case BehaviorState::FAILURE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " FAILURE!";
                    child_node_ptr_->Reset();
                    break;
                default:
                    return;
            }
        }
    };


    class CompositeNode : public BehaviorNode {
    public:
        CompositeNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr) :
                BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
                children_node_index_(0) {
        }

        virtual ~CompositeNode() = default;

        virtual void AddChildren(const BehaviorNode::Ptr &child_node_ptr) {
            children_node_ptr_.push_back(child_node_ptr);
            child_node_ptr->SetParent(shared_from_this());
        }

        virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list) {
            for (auto child_node_ptr = child_node_ptr_list.begin();
                 child_node_ptr != child_node_ptr_list.end(); child_node_ptr++) {
                children_node_ptr_.push_back(*child_node_ptr);
                (*child_node_ptr)->SetParent(shared_from_this());
            }

        }

        std::vector<BehaviorNode::Ptr> GetChildren() {
            return children_node_ptr_;
        }

        unsigned int GetChildrenIndex() {
            return children_node_index_;
        }

        unsigned int GetChildrenNum() {
            return children_node_ptr_.size();
        }

    protected:
        virtual BehaviorState Update() = 0;

        virtual void OnInitialize() = 0;

        virtual void OnTerminate(BehaviorState state) = 0;


        std::vector<BehaviorNode::Ptr> children_node_ptr_;
        unsigned int children_node_index_;


    };

    class SelectorNode : public CompositeNode {
    public:
        SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr) :
                CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr) {
        }

        virtual ~SelectorNode() = default;

        virtual void AddChildren(const BehaviorNode::Ptr &child_node_ptr) {

            CompositeNode::AddChildren(child_node_ptr);

            children_node_reevaluation_.push_back
                    (child_node_ptr->GetBehaviorType() == BehaviorType::PRECONDITION
                     && (std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() ==
                         AbortType::LOW_PRIORITY
                         || std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() ==
                            AbortType::BOTH));

        }

        virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list) {

            CompositeNode::AddChildren(child_node_ptr_list);

            for (auto child_node_ptr = child_node_ptr_list.begin();
                 child_node_ptr != child_node_ptr_list.end(); child_node_ptr++) {
                children_node_reevaluation_.push_back
                        ((*child_node_ptr)->GetBehaviorType() == BehaviorType::PRECONDITION
                         && (std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() ==
                             AbortType::LOW_PRIORITY
                             || std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() ==
                                AbortType::BOTH));
            }
        }

        void SetChildrenIndex(unsigned int children_node_index) {
            children_node_index_ = children_node_index;
        }

    protected:
        virtual void OnInitialize() {
            children_node_index_ = 0;
            //LOG_INFO << name_ << " " << __FUNCTION__;
        };

        virtual BehaviorState Update() {

            if (children_node_ptr_.size() == 0) {
                return BehaviorState::SUCCESS;
            }

            //Reevaluation
            for (unsigned int index = 0; index < children_node_index_; index++) {
                //LOG_INFO << "Reevaluation";
                if (children_node_reevaluation_.at(index)) {
                    BehaviorState state = children_node_ptr_.at(index)->Run();
                    if (index == children_node_index_) {
                        //LOG_INFO << name_ << " abort goes on! ";
                        if (state != BehaviorState::FAILURE) {
                            return state;
                        }
                        ++children_node_index_;
                        break;
                    }
                }
            }

            // 要是self和none在前面，是不是就错过这次select了？
            while (true) {

                BehaviorState state = children_node_ptr_.at(children_node_index_)->Run();

                if (state != BehaviorState::FAILURE) {
                    return state;
                }

                if (++children_node_index_ == children_node_ptr_.size()) {
                    children_node_index_ = 0;
                    return BehaviorState::FAILURE;
                }

            }
        }

        virtual void OnTerminate(BehaviorState state) {
            //selector返回非running状态，只要不是IDLE，就不会停止子节点的工作
            //如果被reset（因为里面包括了使变IDLE的操作），会停止正在工作的子节点
            switch (state) {
                case BehaviorState::IDLE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " IDLE!";
                    //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
                    children_node_ptr_.at(children_node_index_)->Reset();
                    break;
                case BehaviorState::SUCCESS:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " SUCCESS!";
                    break;
                case BehaviorState::FAILURE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " FAILURE!";
                    break;
                default:
                    //LOG_ERROR << name_ << " " << __FUNCTION__ << " ERROR!";
                    return;
            }
        }

        std::vector<bool> children_node_reevaluation_;//如果是AbortType::LOW_PRIORITY或者AbortType::BOTH为true，否则为false
    };

    class SequenceNode : public CompositeNode {
    public:
        SequenceNode(std::string name, const Blackboard::Ptr &blackboard_ptr) :
                CompositeNode::CompositeNode(name, BehaviorType::SEQUENCE, blackboard_ptr) {}

        virtual ~SequenceNode() = default;

    protected:
        virtual void OnInitialize() {
            children_node_index_ = 0;
            //LOG_INFO << name_ << " " << __FUNCTION__;
        };

        virtual BehaviorState Update() {

            if (children_node_ptr_.size() == 0) {
                return BehaviorState::SUCCESS;
            }

            while (true) {

                BehaviorState state = children_node_ptr_.at(children_node_index_)->Run();

                if (state != BehaviorState::SUCCESS) {
                    return state;
                }
                if (++children_node_index_ == children_node_ptr_.size()) {
                    children_node_index_ = 0;
                    return BehaviorState::SUCCESS;
                }

            }
        }

        //序列中的某个子节点失败，就重置，直到该子节点成功为止，并开始运行下一个子节点
        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " IDLE!";
                    children_node_ptr_.at(children_node_index_)->Reset();
                    break;
                case BehaviorState::SUCCESS:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " SUCCESS!";
                    break;
                case BehaviorState::FAILURE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " FAILURE!";
                    break;
                default:
                    //LOG_ERROR << name_ << " " << __FUNCTION__ << " ERROR!";
                    return;
            }
        }
    };

    class ParallelNode : public CompositeNode {
    public:
        ParallelNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                     unsigned int threshold) :
                CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr),
                threshold_(threshold),
                success_count_(0),
                failure_count_(0) {}

        virtual ~ParallelNode() = default;

    protected:

        virtual void OnInitialize() {
            failure_count_ = 0;
            success_count_ = 0;
            children_node_done_.clear();
            children_node_done_.resize(children_node_ptr_.size(), false);
            //LOG_INFO << name_ << " " << __FUNCTION__;
        };

        //失败子节点或成功子节点高于阈值，就OnTerminate并行节点
        virtual BehaviorState Update() {

            if (children_node_ptr_.size() == 0) {
                return BehaviorState::SUCCESS;
            }

            for (unsigned int index = 0; index != children_node_ptr_.size(); index++) {
                if (children_node_done_.at(index) == false) {
                    BehaviorState state = children_node_ptr_.at(index)->Run();

                    if (state == BehaviorState::SUCCESS) {
                        children_node_done_.at(index) = true;
                        if (++success_count_ >= threshold_) {
                            return BehaviorState::SUCCESS;
                        }
                    } else if (state == BehaviorState::FAILURE) {
                        children_node_done_.at(index) = true;
                        if (++failure_count_ >= children_node_ptr_.size() - threshold_) {
                            return BehaviorState::FAILURE;
                        }
                    }

                }
            }
            return BehaviorState::RUNNING;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " IDLE!";
                    break;
                case BehaviorState::SUCCESS:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " SUCCESS!";
                    break;
                case BehaviorState::FAILURE:
                    //LOG_INFO << name_ << " " << __FUNCTION__ << " FAILURE!";
                    break;
                default:
                    //LOG_ERROR << name_ << " " << __FUNCTION__ << " ERROR!";
                    return;
            }
            //TODO: no matter what state, the node would reset all running children to terminate.
            for (unsigned int index = 0; index != children_node_ptr_.size(); index++) {
                children_node_ptr_.at(index)->Reset();
            }
        };
        std::vector<bool> children_node_done_;
        unsigned int success_count_;
        unsigned int failure_count_;
        unsigned int threshold_;
    };

    bool PreconditionNode::Reevaluation() {

        // Back Reevaluation
        if (parent_node_ptr_ != nullptr && parent_node_ptr_->GetBehaviorType() == BehaviorType::SELECTOR
            && (abort_type_ == AbortType::LOW_PRIORITY || abort_type_ == AbortType::BOTH)) {
            auto parent_selector_node_ptr = std::dynamic_pointer_cast<SelectorNode>(parent_node_ptr_);

            auto parent_children = parent_selector_node_ptr->GetChildren();
            // 在父节点（selector）中找自己
            auto iter_in_parent = std::find(parent_children.begin(), parent_children.end(), shared_from_this());
            if (iter_in_parent == parent_children.end()) {
                //LOG_ERROR << "Can't find current node in parent!";
                return false;
            }
            unsigned int index_in_parent = iter_in_parent - parent_children.begin();

            if (index_in_parent < parent_selector_node_ptr->GetChildrenIndex()) {
                if (Precondition()) {
                    //Abort Measures
                    //LOG_INFO << "Abort happens!" << std::endl;
                    parent_children.at(parent_selector_node_ptr->GetChildrenIndex())->Reset();
                    parent_selector_node_ptr->SetChildrenIndex(index_in_parent);
                    return true;
                } else {
                    return false;
                }
            }
        }
        // Self Reevaluation

        if (abort_type_ == AbortType::SELF || abort_type_ == AbortType::BOTH
            || child_node_ptr_->GetBehaviorState() != BehaviorState::RUNNING) {
            if (!Precondition()) {
                //那个AbortType::None真麻烦，意思就是，它下面的节点RUNNING状态下无法停止
                //值得一提的是  LOW_PRIORITY，也是这样
                return false;
            }
        }
        return true;
    }

} //namespace decision

#endif //MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H
