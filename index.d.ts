

///<reference types="./box2d.d.ts" />

declare namespace zim {

    export class Physics {
        constructor(gravity?: number, borders?: zim.Boundary | string, scroll?: boolean, frame?: zim.Frame);
        borders(boundary?:zim.Boundary|string):this 
        drag(array?:[zim.DisplayObject]):this 
        noDrag(array?:[zim.DisplayObject]):this 
        pause(type?:boolean):this  
        join(obj1:zim.DisplayObject, obj2:zim.DisplayObject, point1?:zim.Point, point2?:zim.Point, minAngle?:number, maxAngle?:number, type?:string):Box2D.b2Joint  
        break(joint:Box2D.b2Joint):void 
        attach(control:zim.DisplayObject, obj:zim.DisplayObject):any
        unattach(id:any):void
        debug():this
        updateDebug():this
        removeDebug():this
        dispose():boolean

        readonly world:Box2D.world
        readonly frame:zim.Frame
        readonly scale:number
        readonly step:number
        timeStep:number
        gravity:number
        readonly paused:boolean
        scroll:boolean
        readonly ticker:Function 
        readonly controlObj:zim.DisplayObject
        readonly followObj:zim.DisplayObject
        readonly borderTop:Box2D.body
        readonly borderBottom:Box2D.body
        readonly borderLeft:Box2D.body
        readonly borderRight:Box2D.body
    }
}

export = zim
