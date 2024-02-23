

///<reference types="./box2d.d.ts" />

import {Boundary, Frame, Point, DisplayObject } from "zimjs"

declare namespace zim {

    export class Physics {
        constructor(config_or_gravity?: number, borders?: Boundary | string, scroll?: boolean, frame?: Frame);
        constructor(config:{gravity?: number, borders?: Boundary | string, scroll?: boolean, frame?: Frame});
        borders(boundary?:Boundary|string):this 
        drag(array?:[DisplayObject]):this 
        noDrag(array?:[DisplayObject]):this 
        pause(type?:boolean):this  
        join(obj1:DisplayObject, obj2:DisplayObject, point1?:Point, point2?:Point, minAngle?:number, maxAngle?:number, type?:string):Box2D.b2Joint  
        break(joint:Box2D.b2Joint):void 
        attach(control:DisplayObject, obj:DisplayObject):any
        unattach(id:any):void
        debug():this
        updateDebug():this
        removeDebug():this
        dispose():boolean

        readonly world:Box2D.b2World
        readonly frame:Frame
        readonly scale:number
        readonly step:number
        timeStep:number
        gravity:number
        readonly paused:boolean
        scroll:boolean
        readonly ticker:Function 
        readonly controlObj:DisplayObject
        readonly followObj:DisplayObject
        readonly borderTop:Box2D.b2Body
        readonly borderBottom:Box2D.b2Body
        readonly borderLeft:Box2D.b2Body
        readonly borderRight:Box2D.b2Body
    }
}

export = zim
