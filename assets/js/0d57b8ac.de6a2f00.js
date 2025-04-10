"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[981],{7419:(e,n,o)=>{o.r(n),o.d(n,{assets:()=>l,contentTitle:()=>a,default:()=>h,frontMatter:()=>s,metadata:()=>t,toc:()=>c});const t=JSON.parse('{"id":"programming/threads","title":"Multi-Tasking (Parallel Computing)","description":"Now here\'s where coding gets a lot more complicated. Typically once you start being very competitive, you want to increase the speed of your robot. The easiest way that this can be done by having the robot now do multiple things at once. In computing terms this is refered to as Asynchrony or Parallel Computing (Often used interchangably).","source":"@site/docs/programming/threads.md","sourceDirName":"programming","slug":"/programming/threads","permalink":"/9457_Library/docs/programming/threads","draft":false,"unlisted":false,"tags":[],"version":"current","sidebarPosition":6,"frontMatter":{"title":"Multi-Tasking (Parallel Computing)","sidebar_position":6},"sidebar":"programmingSidebar","previous":{"title":"Auton Code","permalink":"/9457_Library/docs/programming/autoncode"},"next":{"title":"PID\'s (Feedback Control Systems)","permalink":"/9457_Library/docs/programming/PID"}}');var i=o(4848),r=o(8453);const s={title:"Multi-Tasking (Parallel Computing)",sidebar_position:6},a="Multi-Tasking (Parallel Computing)",l={},c=[{value:"Built-in Motor Functions",id:"built-in-motor-functions",level:2},{value:"Blocking/Non-Blocking Functions",id:"blockingnon-blocking-functions",level:3},{value:"Threads/Tasks",id:"threadstasks",level:2}];function d(e){const n={code:"code",em:"em",h1:"h1",h2:"h2",h3:"h3",header:"header",li:"li",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,r.R)(),...e.components};return(0,i.jsxs)(i.Fragment,{children:[(0,i.jsx)(n.header,{children:(0,i.jsx)(n.h1,{id:"multi-tasking-parallel-computing",children:"Multi-Tasking (Parallel Computing)"})}),"\n",(0,i.jsxs)(n.p,{children:["Now here's where coding gets a lot more complicated. Typically once you start being very competitive, you want to increase the speed of your robot. The easiest way that this can be done by having the robot now do multiple things at once. In computing terms this is refered to as ",(0,i.jsx)(n.em,{children:"Asynchrony"})," or ",(0,i.jsx)(n.em,{children:"Parallel Computing"})," (Often used interchangably)."]}),"\n",(0,i.jsx)(n.p,{children:"Up to this point you are likely used to code running line-by-line, where a line must complete before moving onto the next one - since most functions operate in a similar way, often times when programmed with this methodology, this means that a drive function must wait for an arm raising function to complete or vise-versa. That super simple example can be illustrated below:"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-c",children:"void AutoControl(void) {\r\n    driveFunction(700, 60);         // The robot will rotate wheels forward by 700 degrees at 60 pct motor speed.\r\n    armMotor.spintoPosition(135);   // Waiting for driveFunction to finish, The arm will then rotate 135 degrees\r\n}\r\n\r\n// --- OR --- \r\n\r\nvoid AutoControl(void) {\r\n    armMotor.spintoPosition(135);   // The arm will rotate 135 degrees\r\n    driveFunction(700, 60);         // Waiting for spintoPosition to finish, The robot will then rotate wheels forward by 700 degrees at 60 pct motor speed.\r\n}\n"})}),"\n",(0,i.jsx)(n.h2,{id:"built-in-motor-functions",children:"Built-in Motor Functions"}),"\n",(0,i.jsxs)(n.p,{children:["To ease you into the concepts, we can start with some of the built-in commands. Luckily for us, the Motor class actually comes with some built-in parallezation. This can be seen in the ",(0,i.jsx)(n.code,{children:"spintoPosition()"})," function and the ",(0,i.jsx)(n.code,{children:"spinFor()"})," function."]}),"\n",(0,i.jsx)(n.h3,{id:"blockingnon-blocking-functions",children:"Blocking/Non-Blocking Functions"}),"\n",(0,i.jsxs)(n.p,{children:["When functions are required to wait for another to finish, they are called ",(0,i.jsx)(n.strong,{children:"Blocking"})," Functions. Similarly, when they do not have to wait for another function to finish they are called ",(0,i.jsx)(n.strong,{children:"Non-Blocking"}),"."]}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-c",children:"Motor.spinFor(fwd, 45);        // This command is Blocking by default\r\nMotor.spinFor(fwd, 45, true);  // This command is now Blocking\r\nMotor.spinFor(fwd, 45, false); // This command is now non-Blocking\r\n\r\nMotor.spintoPosition(45);        // This command is Blocking by default\r\nMotor.spintoPosition(45, true);  // This command is now Blocking\r\nMotor.spintoPosition(45, false); // This command is now non-Blocking\n"})}),"\n",(0,i.jsx)(n.p,{children:"Now that we know that Non-Blocking functions exist for motors (and motor groups), we now can implement some easy parallazation. Here is that example before, but now the arm will lift, while the robot is driving forward."}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-c",children:"void AutoControl(void) {\r\n    armMotor.spintoPosition(135, false);   // Now it is non-blocking, The arm will rotate 135 degrees\r\n    driveFunction(700, 60);                // No waiting, The robot will then rotate wheels forward by 700 degrees at 60 pct motor speed.\r\n}\n"})}),"\n",(0,i.jsx)(n.p,{children:"While this can be very helpful, it is important to note some of the downfalls!"}),"\n",(0,i.jsxs)(n.ul,{children:["\n",(0,i.jsx)(n.li,{children:"More difficulty in programming autonomous"}),"\n",(0,i.jsx)(n.li,{children:"Could instigate consistency issues"}),"\n",(0,i.jsx)(n.li,{children:"Motors may not behave as you want them to while en-route."}),"\n"]}),"\n",(0,i.jsx)(n.h2,{id:"threadstasks",children:"Threads/Tasks"})]})}function h(e={}){const{wrapper:n}={...(0,r.R)(),...e.components};return n?(0,i.jsx)(n,{...e,children:(0,i.jsx)(d,{...e})}):d(e)}},8453:(e,n,o)=>{o.d(n,{R:()=>s,x:()=>a});var t=o(6540);const i={},r=t.createContext(i);function s(e){const n=t.useContext(r);return t.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function a(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:s(e.components),t.createElement(r.Provider,{value:n},e.children)}}}]);