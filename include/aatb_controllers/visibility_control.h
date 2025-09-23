
#ifndef AATB_CONTROLLERS__VISIBILITY_CONTROL_H_
#define AATB_CONTROLLERS__VISIBILITY_CONTROL_H_

// Linux visibility macros
#define AATB_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define AATB_CONTROLLERS_IMPORT
#define AATB_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define AATB_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#define AATB_CONTROLLERS_PUBLIC_TYPE

#endif  // AATB_CONTROLLERS__VISIBILITY_CONTROL_H_