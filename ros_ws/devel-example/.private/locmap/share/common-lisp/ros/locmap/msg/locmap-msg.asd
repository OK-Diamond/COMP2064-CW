
(cl:in-package :asdf)

(defsystem "locmap-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LocMapGoto" :depends-on ("_package_LocMapGoto"))
    (:file "_package_LocMapGoto" :depends-on ("_package"))
    (:file "LocMapLocation" :depends-on ("_package_LocMapLocation"))
    (:file "_package_LocMapLocation" :depends-on ("_package"))
    (:file "LocMapLocations" :depends-on ("_package_LocMapLocations"))
    (:file "_package_LocMapLocations" :depends-on ("_package"))
  ))