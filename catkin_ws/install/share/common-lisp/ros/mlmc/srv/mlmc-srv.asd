
(cl:in-package :asdf)

(defsystem "mlmc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RunTest" :depends-on ("_package_RunTest"))
    (:file "_package_RunTest" :depends-on ("_package"))
  ))