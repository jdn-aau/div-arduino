## Proto threads

My local copy of [this page](http://dunkels.com/adam/pt)

snip from his HP:

## Protothreads
Protothreads are extremely lightweight stackless threads designed for severely memory constrained systems, such as small embedded systems or wireless sensor network nodes. Protothreads provide linear code execution for event-driven systems implemented in C. Protothreads can be used with or without an underlying operating system to provide blocking event-handlers. Protothreads provide sequential flow of control without complex state machines or full multi-threading.

```C
#include "pt.h"
 
struct pt pt;
struct timer timer;
 
PT_THREAD(example(struct pt *pt))
{
  PT_BEGIN(pt);
 
  while(1) {
    if(initiate_io()) {
      timer_start(&timer);
      PT_WAIT_UNTIL(pt,
         io_completed() ||
         timer_expired(&timer));
      read_data();
    }
  }
  PT_END(pt);
}
```

Example protothreads code.
While protothreads originally were created for memory-constrained embedded systems, it has found many uses as a general purpose library too. Examples include multimedia streaming server software, grid computing research software, and MPEG decoding software for Internet TVs.

Read more...

Main features:

Very small RAM overhead - only two bytes per protothread and no extra stacks
Highly portable - the protothreads library is 100% pure C and no architecture specific assembly code
Can be used with or without an OS
Provides blocking wait without full multi-threading or stack-switching
Freely available under a BSD-like open source license
Example applications:

Memory constrained systems
Event-driven protocol stacks
Small embedded systems
Sensor network nodes
Portable C applications
For example usages, see the Examples page.

The protothreads library is released under an open source BSD-style license that freely allows for both non-commercial and commercial usage. The only requirement is that credit is given. Download the full source code here.

Protothreads were invented by Adam Dunkels with support from Oliver Schmidt <ol.sc@web.de>.


