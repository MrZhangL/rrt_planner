
(cl:in-package :asdf)

(defsystem "robot_move-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "moveAction" :depends-on ("_package_moveAction"))
    (:file "_package_moveAction" :depends-on ("_package"))
    (:file "moveActionFeedback" :depends-on ("_package_moveActionFeedback"))
    (:file "_package_moveActionFeedback" :depends-on ("_package"))
    (:file "moveActionGoal" :depends-on ("_package_moveActionGoal"))
    (:file "_package_moveActionGoal" :depends-on ("_package"))
    (:file "moveActionResult" :depends-on ("_package_moveActionResult"))
    (:file "_package_moveActionResult" :depends-on ("_package"))
    (:file "moveFeedback" :depends-on ("_package_moveFeedback"))
    (:file "_package_moveFeedback" :depends-on ("_package"))
    (:file "moveGoal" :depends-on ("_package_moveGoal"))
    (:file "_package_moveGoal" :depends-on ("_package"))
    (:file "moveResult" :depends-on ("_package_moveResult"))
    (:file "_package_moveResult" :depends-on ("_package"))
  ))