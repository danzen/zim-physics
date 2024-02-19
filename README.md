
<a href="https://zimjs.com/physics" target=_blank>![physics](https://github.com/danzen/zim-physics/assets/380281/0380b4a2-9bc2-417a-98b9-31d9aaf73be6)</a>

<p>ZIM Physics is a helper module for the ZIM JavaScript Canvas Framework that works with Box2DWeb.  
In ZIM TEN we integrated physics - see https://zimjs.com/physics - so it is as easy as:</p>

```JavaScript
new Circle().center().addPhysics();
```

<p>and the circle will drop to the ground! Usually we create a Physics object first so we can set gravity and boundaries if needed and call various methods on the physics object such as drag.  See the ZIM Docs at https://zimjs.com/docs.html?item=Physics and https://zimjs.com/docs.html?item=addPhysics.
</p>

```JavaScript
const physics = new Physics();
physics.drag(); // will make dynamic objects draggable
new Circle().center().addPhysics({bounciness:.7}); 
```

<h2>CDN</h2>
Usually we use ES Modules to bring in ZIM and if we want Physics then we the code below - see the starting template at the top of the https://zimjs.com/code page.

```JavaScript
import zim from "https://zimjs.org/cdn/016/zim_physics";
```

<h2>NPM</h2>
This repository holds the NPM package so you can install from <a href=https://www.npmjs.com/package/@zimjs/physics target=node>@zimjs/physics</a> on NPM.  It includes typings and loads the Box2DWeb package as a dependency.  The <a href=https://www.npmjs.com/package/zimjs target=node>ZIM&nbsp;package</a> must be installed to work.

<h2>PHYSICS EXAMPLES</h2>

<li><a href="https://zimjs.com/physics/" target="b">ZIM with Box2D - PHYSICS</a> - ZIM TEN Example</li>
<li><a href="https://zimjs.com/physics/goal.html" target="b2">ZIM with Box2D - GOAL</a> - ZIM TEN Example</li>
<li><a href="https://zimjs.com/physics/beads.html" target="b3">ZIM with Box2D - BEADS</a> - ZIM TEN Example</li>
<li><a href="https://zimjs.com/physics/keepup.html" target="b4">ZIM with Box2D - KEEP UP</a> - ZIM TEN Example</li>
<li><a href="https://zimjs.com/physics/drive.html" target="b5">ZIM with Box2D - DRIVE</a> - ZIM TEN Example</li>
<li><a href="https://zimjs.com/014/cat.html" target="b5">Cat Food - Physics Blobs</a> - ZIM 014 Example</li>
<li><a href="https://zimjs.com/014/fermenti.html" target="b5">Fermenti - Physics Blobs</a> - ZIM 014 Example</li>
<li><a href="https://zimjs.com/bits/view/physics.html" target="b">ZIM with Box2D</a> - ZIM Bits</li>
<li><a href="https://zimjs.com/bits/view/physics2.html" target="b2">Soup with Box2D</a> - ZIM Bits</li>
<li><a href="https://zimjs.org/kids/bug_bounce.html" target="b">Bugs - three levels</a> - ZIM Kids</li>
<li><a href="https://codepen.io/danzen/pen/EMBQog" target="b">Falling Apple</a> - CodePen (simple)</li>
<li><a href="https://zimjs.com/droid2/index.html" target="b">Alone Droid 2</a> - ZIM (complex)</li>
<li><a href="https://zimjs.com/droid/" target="b">Alone Droid</a> - ZIM (complex)</li>
<li><a href="https://codepen.io/danzen/pen/dybQVoa" target="b">Pinball Sparks</a> - CodePen</li>
<li><a href="https://codepen.io/danzen/pen/agbbRv" target="b">Noodles Hair</a> - CodePen</li>
<li><a href="https://codepen.io/zimjs/pen/wboRbE" target="b">Anti-grav</a> - CodePen</li>
<li><a href="https://codepen.io/danzen/pen/gEbXVm" target="b">Polygon Pen</a> - CodePen</li>
<li><a href="https://codepen.io/danzen/pen/daJjpr" target="b">Data Vis</a> - CodePen</li>
<li><a href="https://codepen.io/danzen/pen/ROPxNQ" target="b">Maze</a> - CodePen</li>
<li><a href="https://zimjs.com/sidescroller" target="b">SideScroller</a> - ZIM</li>
<li><a href="https://zimjs.com/valentines/puppets.html" target="b">Valentines Puppets</a> - ZIM</li>


<h2>VIDEOS</h2>
CODE IN FIVE (See <a href="https://zimjs.com/five/" target="fm">https://zimjs.com/five/</a> for files)
<li><a href="https://www.youtube.com/watch?v=m7GYqgPE5Ik" target="fm">Follow</a> - Code in Five Minutes</li>
<li><a href="https://www.youtube.com/watch?v=LPJYULQd_h0" target="fm">Shoot</a> - Code in Five Minutes</li>
<li><a href="https://www.youtube.com/watch?v=ATfMF25i608" target="fm">Angry Birds A</a> - Code in Five Minutes</li>
<li><a href="https://www.youtube.com/watch?v=xe8k5ts-Ufk" target="fm">Angry Birds B</a> - Code in Five Minutes</li>
<li><a href="https://www.youtube.com/watch?v=NUHIniq6p3c" target="fm">Angry Birds C</a> - Code in Five Minutes</li>
<li><a href="https://www.youtube.com/watch?v=PFiP1SzBNFE" target="fm">Angry Birds D</a> - Code in Five Minutes</li>


<h2>ZIM</h2>
See the ZIM repository at https://github.com/danzen/zimjs for information on ZIM and open source license, etc.
