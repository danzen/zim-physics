// https://www.npmjs.com/package/@ludic/box2d
declare namespace Box2D {

  export interface Box2DConfig {
    locateFile: (path: string)=>string
  }

  export class b2Contact {
    GetManifold(): b2Manifold
    GetWorldManifold(manifold: b2WorldManifold): void
    IsTouching(): boolean
    SetEnabled(flag: boolean): void
    IsEnabled(): boolean
    GetNext(): b2Contact
    GetFixtureA(): b2Fixture
    GetChildIndexA(): number
    GetFixtureB(): b2Fixture
    GetChildIndexB(): number
    SetFriction(friction: number): void
    GetFriction(): number
    ResetFriction(): void
    SetRestitution(restitution: number): void
    GetRestitution(): number
    ResetRestitution(): void
    SetTangentSpeed(speed: number): void
    GetTangentSpeed(): number
  }

  export class b2ContactListener {
  }

  // [JSImplementation="b2ContactListener"]
  export class JSContactListener extends b2ContactListener {
    constructor()

    BeginContact(contact: b2Contact): void
    EndContact(contact: b2Contact): void
    // TODO: Declare another b2ContactListener implementation without PreSolve/PostSolve,
    // for efficiency (otherwise the JS implementations of these functions must get called
    // even if unused).
    PreSolve(contact: b2Contact, oldManifold: b2Manifold): void
    PostSolve(contact: b2Contact, impulse: b2ContactImpulse): void
  }

  export class b2World {
    constructor(gravity: b2Vec2)
    SetDestructionListener(listener: b2DestructionListener): void
    SetContactFilter(filter: JSContactFilter): void
    SetContactListener(listener: JSContactListener): void
    SetDebugDraw(debugDraw: b2Draw): void
    CreateBody(def: b2BodyDef): b2Body
    DestroyBody(body: b2Body): void
    CreateJoint(def: b2JointDef): b2Joint
    DestroyJoint(joint: b2Joint): void
    Step(timeStep: number, velocityIterations: number, positionIterations: number): void
    ClearForces(): void
    DrawDebugData(): void
    QueryAABB(callback: b2QueryCallback, aabb: b2AABB): void
    RayCast(callback: b2RayCastCallback, point1: b2Vec2, point2: b2Vec2): void
    GetBodyList(): b2Body
    GetJointList(): b2Joint
    GetContactList(): b2Contact
    SetAllowSleeping(flag: boolean): void
    GetAllowSleeping(): boolean
    SetWarmStarting(flag: boolean): void
    GetWarmStarting(): boolean
    SetContinuousPhysics(flag: boolean): void
    GetContinuousPhysics(): boolean
    SetSubStepping(flag: boolean): void
    GetSubStepping(): boolean
    GetProxyCount(): number
    GetBodyCount(): number
    GetJointCount(): number
    GetContactCount(): number
    GetTreeHeight(): number
    GetTreeBalance(): number
    GetTreeQuality(): number
    SetGravity(gravity: b2Vec2): void
    GetGravity(): b2Vec2
    IsLocked(): boolean
    SetAutoClearForces(flag: boolean): void
    GetAutoClearForces(): boolean
    GetProfile(): b2Profile
    Dump(): void
  }

  export enum b2ShapeType {
    circle,
    edge,
    polygon,
    chain,
    typeCount
  }

  export class b2Shape {
    GetType(): b2ShapeType
    GetChildCount(): number
    TestPoint(xf: b2Transform, p: b2Vec2): boolean
    RayCast(output: b2RayCastOutput, input: b2RayCastInput, transform: b2Transform, childIndex: number): boolean
    ComputeAABB(aabb: b2AABB, xf: b2Transform, childIndex: number): void
    ComputeMass(massData: b2MassData, density: number): void

    m_type: b2ShapeType
    m_radius: number
  }

  export class b2FixtureDef {
    constructor()
    shape: b2Shape
    userData: any
    friction: number
    restitution: number
    density: number
    isSensor: boolean
    filter: b2Filter
  }

  export class b2Fixture {
    GetType(): b2ShapeType
    GetShape(): b2Shape
    SetSensor(sensor: boolean): void
    IsSensor(): boolean
    SetFilterData(filter: b2Filter): void
    GetFilterData(): b2Filter
    Refilter(): void
    GetBody(): b2Body
    GetNext(): b2Fixture
    GetUserData(): any
    SetUserData(data: any): void
    TestPoint(p: b2Vec2): boolean
    RayCast(output: b2RayCastOutput, input: b2RayCastInput, childIndex: number): boolean
    GetMassData(massData: b2MassData): void
    SetDensity(density: number): void
    GetDensity(): number
    GetFriction(): number
    SetFriction(friction: number): void
    GetRestitution(): number
    SetRestitution(restitution: number): void
    GetAABB(childIndex: number): b2AABB
    Dump(bodyIndex: number): void
  }

  export class b2Transform {
    constructor()
    constructor(position: b2Vec2, rotation: b2Rot)
    SetIdentity(): void
    Set(position: b2Vec2, angle: number): void
    p: b2Vec2
    q: b2Rot
  }


  export class b2RayCastCallback {
  }
  // [JSImplementation="b2RayCastCallback"]
  export class JSRayCastCallback extends b2RayCastCallback {
    constructor()
    ReportFixture(fixture: b2Fixture, point: b2Vec2, normal: b2Vec2, fraction: number): number
  }

  export class b2QueryCallback {
  }
  // [JSImplementation="b2QueryCallback"]
  export class JSQueryCallback extends b2QueryCallback {
    constructor()
    ReportFixture(fixture: b2Fixture): boolean
  }

  export class b2MassData {
    constructor()
    mass: number
    center: b2Vec2
    I: number
  }

  export class b2Vec2 {
    x: number
    y: number
    constructor()
    constructor(x: number, y: number)
    SetZero(): void
    Set(x: number, y: number): void
    // [Operator="+="]
    op_add(v: b2Vec2): void
    // [Operator="-="]
    op_sub(v: b2Vec2): void
    // [Operator="*="]
    op_mul(s: number): void
    Length(): number
    LengthSquared(): number
    Normalize(): number
    IsValid(): boolean
    // [Value]
    Skew(): b2Vec2
  }

  export class b2Vec3 {
    constructor()
    constructor(x: number, y: number, z: number)
    SetZero(): void
    Set(x: number, y: number, z: number): void
    // [Operator="+="]
    op_add(v: b2Vec3): void
    // [Operator="-="]
    op_sub(v: b2Vec3): void
    // [Operator="*="]
    op_mul(s: number): void
    x: number
    y: number
    z: number
  }

  // [NoDelete]
  export class b2Body {
    CreateFixture(def: b2FixtureDef): b2Fixture
    CreateFixture(shape: b2Shape, density: number): b2Fixture
    DestroyFixture(fixture: b2Fixture): void
    SetTransform(position: b2Vec2, angle: number): void
    GetTransform(): b2Transform
    GetPosition(): b2Vec2
    GetAngle(): number
    GetWorldCenter(): b2Vec2
    GetLocalCenter(): b2Vec2
    SetLinearVelocity(v: b2Vec2): void
    GetLinearVelocity(): b2Vec2
    SetAngularVelocity(omega: number): void
    GetAngularVelocity(): number
    ApplyForce(force: b2Vec2, point: b2Vec2, awake: boolean): void
    ApplyForceToCenter(force: b2Vec2, awake: boolean): void
    ApplyTorque(torque: number, awake: boolean): void
    ApplyLinearImpulse(impulse: b2Vec2, point: b2Vec2, awake: boolean): void
    ApplyAngularImpulse(impulse: number, awake: boolean): void
    GetMass(): number
    GetInertia(): number
    GetMassData(data: b2MassData): void
    SetMassData(data: b2MassData): void
    ResetMassData(): void
    GetWorldPoint(localPoint: b2Vec2): b2Vec2
    GetWorldVector(localVector: b2Vec2): b2Vec2
    GetLocalPoint(worldPoint: b2Vec2): b2Vec2
    GetLocalVector(worldVector: b2Vec2): b2Vec2
    GetLinearVelocityFromWorldPoint(worldPoint: b2Vec2): b2Vec2
    GetLinearVelocityFromLocalPoint(localPoint: b2Vec2): b2Vec2
    GetLinearDamping(): number
    SetLinearDamping(linearDamping: number): void
    GetAngularDamping(): number
    SetAngularDamping(angularDamping: number): void
    GetGravityScale(): number
    SetGravityScale(scale: number): void
    SetType(type: b2BodyType): void
    GetType(): b2BodyType
    SetBullet(flag: boolean): void
    IsBullet(): boolean
    SetSleepingAllowed(flag: boolean): void
    IsSleepingAllowed(): boolean
    SetAwake(flag: boolean): void
    IsAwake(): boolean
    SetActive(flag: boolean): void
    IsActive(): boolean
    SetFixedRotation(flag: boolean): void
    IsFixedRotation(): boolean
    GetFixtureList(): b2Fixture
    GetJointList(): b2JointEdge
    GetContactList(): b2ContactEdge
    GetNext(): b2Body
    GetUserData(): any
    SetUserData(data: any): void
    GetWorld(): b2World
    Dump(): void
  }

  export enum b2BodyType {
    static,
    kinematic,
    dynamic
  }

  export class b2BodyDef {
    constructor()

    type: b2BodyType
    position: b2Vec2
    angle: number
    linearVelocity: b2Vec2
    angularVelocity: number
    linearDamping: number
    angularDamping: number
    allowSleep: boolean
    awake: boolean
    fixedRotation: boolean
    bullet: boolean
    active: boolean
    userData: any
    gravityScale: number
  }

  export class b2Filter {
    constructor()
    categoryBits: number
    maskBits: number
    groupIndex: number
  }

  export class b2AABB {
    constructor()
    IsValid(): boolean
    GetCenter(): b2Vec2
    GetExtents(): b2Vec2
    GetPerimeter(): number
    Combine(aabb: b2AABB): void
    Combine(aabb1: b2AABB, aabb2: b2AABB): void
    Contains(aabb: b2AABB): boolean
    RayCast(output: b2RayCastOutput, input: b2RayCastInput): boolean
    lowerBound: b2Vec2
    upperBound: b2Vec2
  }

  // b2CircleShape implements b2Shape
  export class b2CircleShape extends b2Shape {
    constructor()
    m_p: b2Vec2
  }
  
  // b2EdgeShape implements b2Shape
  export class b2EdgeShape extends b2Shape {
    constructor()
    Set(v1: b2Vec2, v2: b2Vec2): void

    m_vertex1: b2Vec2
    m_vertex2: b2Vec2
    m_vertex0: b2Vec2
    m_vertex3: b2Vec2
    m_hasVertex0: boolean
    m_hasVertex3: boolean
  }

  export enum b2JointType {
    unknown,
    revolute,
    prismatic,
    distance,
    pulley,
    mouse,
    gear,
    wheel,
    weld,
    friction,
    rope,
    motor
  }

  export enum b2LimitState {
    inactive,
    atLower,
    atUpper,
    equal
  }

  export abstract class b2JointDef {
    constructor()
    type: b2JointType
    userData: any
    bodyA: b2Body
    bodyB: b2Body
    collideConnected: boolean
  }

  // [NoDelete]
  export abstract class b2Joint {
    GetType(): b2JointType
    GetBodyA(): b2Body
    GetBodyB(): b2Body
    GetAnchorA(): b2Vec2
    GetAnchorB(): b2Vec2
    GetReactionForce(inv_dt: number): b2Vec2
    GetReactionTorque(inv_dt: number): number
    GetNext(): b2Joint
    GetUserData(): any
    SetUserData(data: any): void
    IsActive(): boolean
    GetCollideConnected(): boolean
    Dump(): void
  }

  // b2WeldJoint implements b2Joint
  export class b2WeldJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    SetFrequency(hz: number): void
    GetFrequency(): number
    SetDampingRatio(ratio: number): void
    GetDampingRatio(): number
    Dump(): void
  }

  // b2WeldJointDef implements b2JointDef
  export class b2WeldJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2): void
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    referenceAngle: number
    frequencyHz: number
    dampingRatio: number
  }

  // b2ChainShape implements b2Shape
  export class b2ChainShape extends b2Shape {
    constructor()
    Clear(): void
    CreateLoop(vertices: b2Vec2, count: number): void
    CreateChain(vertices: b2Vec2, count: number): void
    SetPrevVertex(prevVertex: b2Vec2): void
    SetNextVertex(nextVertex: b2Vec2): void
    GetChildEdge(edge: b2EdgeShape, index: number): void

    m_vertices: b2Vec2
    m_count: number
    m_prevVertex: b2Vec2
    m_nextVertex: b2Vec2
    m_hasPrevVertex: boolean
    m_hasNextVertex: boolean
  }

  export class b2Color {
    constructor()
    constructor(r: number, g: number, b: number)
    Set(ri: number, gi: number, bi: number): void

    r: number
    g: number
    b: number
  }

  export class b2ContactEdge {
    constructor()
    other: b2Body
    contact: b2Contact
    prev: b2ContactEdge
    next: b2ContactEdge
  }

  export enum b2ContactFeatureType {
    vertex,
    face
  }

  export interface b2ContactFeature {
    indexA: number
    indexB: number
    typeA: number
    typeB: number
  }

  export class b2ContactFilter {
  }

  // [JSImplementation="b2ContactFilter"]
  export class JSContactFilter extends b2ContactFilter {
    constructor()
    ShouldCollide(fixtureA: b2Fixture, fixtureB: b2Fixture): boolean
  }

  export interface b2ContactID {
    cf: b2ContactFeature
    key: number
  }

  export interface b2ContactImpulse {
    // TODO: webidl_binder support for array types.
    // normalImpulses: number[]
    // tangentImpulses: number[]
    count: number
  }

  export interface b2DestructionListener {
  }

  export class b2DestructionListenerWrapper {
  }

  // [JSImplementation="b2DestructionListenerWrapper"]
  export class JSDestructionListener extends b2DestructionListenerWrapper {
    constructor()
    // These methods map the overloaded methods from b2DestructionListener onto differently-named
    // methods, so that it is possible to implement both of them in JS.
    SayGoodbyeJoint(joint: b2Joint): void
    SayGoodbyeFixture(joint: b2Fixture): void
  }

  // b2DistanceJoint implements b2Joint
  export class b2DistanceJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    SetLength(length: number): void
    GetLength(): number
    SetFrequency(hz: number): void
    GetFrequency(): number
    SetDampingRatio(ratio: number): void
    GetDampingRatio(): number
  }


  // b2DistanceJointDef implements b2JointDef
  export class b2DistanceJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, anchorA: b2Vec2, anchorB: b2Vec2): void
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    length: number
    frequencyHz: number
    dampingRatio: number
  }


  export enum b2DrawFlag {
    shapeBit,
    jointBit,
    aabbBit,
    pairBit,
    centerOfMassBit
  }

  export class b2Draw {
    SetFlags(flags: number): void
    GetFlags(): number
    AppendFlags(flags: number): void
    ClearFlags(flags: number): void
  }

  // [JSImplementation="b2Draw"]
  export class JSDraw extends b2Draw {
    constructor()
    DrawPolygon(vertices: b2Vec2, vertexCount: number, color: b2Color): void
    DrawSolidPolygon(vertices: b2Vec2, vertexCount: number, color: b2Color): void
    DrawCircle(center: b2Vec2, radius: number, color: b2Color): void
    DrawSolidCircle(center: b2Vec2, radius: number, axis: b2Vec2, color: b2Color): void
    DrawSegment(p1: b2Vec2, p2: b2Vec2, color: b2Color): void
    DrawTransform(xf: b2Transform): void
  }

  // b2FrictionJoint implements b2Joint
  export class b2FrictionJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    SetMaxForce(force: number): void
    GetMaxForce(): number
    SetMaxTorque(torque: number): void
    GetMaxTorque(): number
  }

  // b2FrictionJointDef implements b2JointDef
  export class b2FrictionJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2): void
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    maxForce: number
    maxTorque: number
  }

  // b2GearJoint implements b2Joint
  export class b2GearJoint extends b2Joint {
    GetJoint1(): b2Joint
    GetJoint2(): b2Joint
    SetRatio(ratio: number): void
    GetRatio(): number
  }

  // b2GearJointDef implements b2JointDef
  export class b2GearJointDef extends b2JointDef {
    constructor()
    joint1: b2Joint
    joint2: b2Joint
    ratio: number
  }

  export class b2JointEdge {
    constructor()
    other: b2Body
    joint: b2Joint
    prev: b2JointEdge
    next: b2JointEdge
  }

  export enum b2ManifoldType {
    circles,
    faceA,
    faceB
  }

  export class b2Manifold {
    constructor()
    // TODO: webidl_binder support for array types.
    // points: b2ManifoldPoint[]
    localNormal: b2Vec2
    localPoint: b2Vec2
    type: b2ManifoldType
    pointCount: number
  }

  export class b2WorldManifold {
    constructor()
    Initialize(manifold: b2Manifold, xfA: b2Transform, radiusA: number, xfB: b2Transform, radiusB: number): void
    normal: b2Vec2
    points: b2Vec2[]
    separations: number[]
  }

  export class b2ManifoldPoint {
    constructor()
    localPoint: b2Vec2
    normalImpulse: number
    tangentImpulse: number
    id: b2ContactID
  }

  export class b2Mat22 {
    constructor()
    constructor(c1: b2Vec2, c2: b2Vec2)
    constructor(a11: number, a12: number, a21: number, a22: number)
    Set(c1: b2Vec2, c2: b2Vec2): void
    SetIdentity(): void
    SetZero(): void
    GetInverse(): b2Mat22
    Solve(b: b2Vec2): b2Vec2

    ex: b2Vec2
    ey: b2Vec2
  }

  export class b2Mat33 {
    constructor()
    constructor(c1: b2Vec3, c2: b2Vec3, c3: b2Vec3)
    SetZero(): void
    Solve33(b: b2Vec3): b2Vec3
    Solve22(b: b2Vec2): b2Vec2
    GetInverse22(M: b2Mat33): void
    GetSymInverse33(M: b2Mat33): void

    ex: b2Vec3
    ey: b2Vec3
    ez: b2Vec3
  }

  // b2MouseJoint implements b2Joint
  export class b2MouseJoint extends b2Joint {
    SetTarget(target: b2Vec2): void
    GetTarget(): b2Vec2
    SetMaxForce(force: number): void
    GetMaxForce(): number
    SetFrequency(hz: number): void
    GetFrequency(): number
    SetDampingRatio(ratio: number): void
    GetDampingRatio(): number
  }

  // b2MouseJointDef implements b2JointDef
  export class b2MouseJointDef extends b2JointDef {
    constructor()
    target: b2Vec2
    maxForce: number
    frequencyHz: number
    dampingRatio: number
  }

  // b2PolygonShape implements b2Shape
  export class b2PolygonShape extends b2Shape {
    constructor()
    Set(vertices: b2Vec2, vertexCount: number): void
    SetAsBox(hx: number, hy: number): void
    SetAsBox(hx: number, hy: number, center: b2Vec2, angle: number): void
    GetVertexCount(): number
    GetVertex(index: number): b2Vec2
    m_centroid: b2Vec2

    // TODO: webidl_binder support for array types.
    m_vertices: b2Vec2[]
    m_normals: b2Vec2[]

    m_count: number
  }

  // b2PrismaticJoint implements b2Joint
  export class b2PrismaticJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    GetLocalAxisA(): b2Vec2
    GetReferenceAngle(): number
    GetJointTranslation(): number
    GetJointSpeed(): number
    IsLimitEnabled(): boolean
    EnableLimit(flag: boolean): void
    GetLowerLimit(): number
    GetUpperLimit(): number
    SetLimits(lower: number, upper: number): void
    IsMotorEnabled(): boolean
    EnableMotor(flag: boolean): void
    SetMotorSpeed(speed: number): void
    GetMotorSpeed(): number
    SetMaxMotorForce(force: number): void
    GetMaxMotorForce(): number
    GetMotorForce(inv_dt: number): number
  }

  // b2PrismaticJointDef implements b2JointDef
  export class b2PrismaticJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2, axis: b2Vec2): void
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    localAxisA: b2Vec2
    referenceAngle: number
    enableLimit: boolean
    lowerTranslation: number
    upperTranslation: number
    enableMotor: boolean
    maxMotorForce: number
    motorSpeed: number
  }

  export interface b2Profile {
    step: number
    collide: number
    solve: number
    solveInit: number
    solveVelocity: number
    solvePosition: number
    broadphase: number
    solveTOI: number
  }

  // b2PulleyJoint implements b2Joint
  export class b2PulleyJoint extends b2Joint {
    GetGroundAnchorA(): b2Vec2
    GetGroundAnchorB(): b2Vec2
    GetLengthA(): number
    GetLengthB(): number
    GetRatio(): number
    GetCurrentLengthA(): number
    GetCurrentLengthB(): number
  }

  // b2PulleyJointDef implements b2JointDef
  export class b2PulleyJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, groundAnchorA: b2Vec2, groundAnchorB: b2Vec2, anchorA: b2Vec2, anchorB: b2Vec2, ratio: number): void
    groundAnchorA: b2Vec2
    groundAnchorB: b2Vec2
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    lengthA: number
    lengthB: number
    ratio: number
  }

  export interface b2RayCastInput {
    p1: b2Vec2
    p2: b2Vec2
    maxFraction: number
  }

  export interface b2RayCastOutput {
    normal: b2Vec2
    fraction: number
  }
  
  // b2RevoluteJoint implements b2Joint
  export class b2RevoluteJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    GetReferenceAngle(): number
    GetJointAngle(): number
    GetJointSpeed(): number
    IsLimitEnabled(): boolean
    EnableLimit(flag: boolean): void
    GetLowerLimit(): number
    GetUpperLimit(): number
    SetLimits(lower: number, upper: number): void
    IsMotorEnabled(): boolean
    EnableMotor(flag: boolean): void
    SetMotorSpeed(speed: number): void
    GetMotorSpeed(): number
    SetMaxMotorTorque(torque: number): void
    GetMaxMotorTorque(): number
    GetMotorTorque(inv_dt: number): number
  }

  // b2RevoluteJointDef implements b2JointDef
  export class b2RevoluteJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2): void
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    referenceAngle: number
    enableLimit: boolean
    lowerAngle: number
    upperAngle: number
    enableMotor: boolean
    motorSpeed: number
    maxMotorTorque: number
  }

  // b2RopeJoint implements b2Joint
  export class b2RopeJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    SetMaxLength(length: number): void
    GetMaxLength(): number
    GetLimitState(): b2LimitState
  }

  // b2RopeJointDef implements b2JointDef
  export class b2RopeJointDef extends b2JointDef {
    constructor()
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    maxLength: number
  }

  export class b2Rot {
    constructor()
    constructor(angle: number)
    Set(angle: number): void
    SetIdentity(): void
    GetAngle(): number
    GetXAxis(): b2Vec2
    GetYAxis(): b2Vec2
    s: number
    c: number
  }

  // b2WheelJoint implements b2Joint
  export class b2WheelJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2
    GetLocalAnchorB(): b2Vec2
    GetLocalAxisA(): b2Vec2
    GetJointTranslation(): number
    GetJointSpeed(): number
    IsMotorEnabled(): boolean
    EnableMotor(flag: boolean): void
    SetMotorSpeed(speed: number): void
    GetMotorSpeed(): number
    SetMaxMotorTorque(torque: number): void
    GetMaxMotorTorque(): number
    GetMotorTorque(inv_dt: number): number
    SetSpringFrequencyHz(hz: number): void
    GetSpringFrequencyHz(): number
    SetSpringDampingRatio(ratio: number): void
    GetSpringDampingRatio(): number
  }

  // b2WheelJointDef implements b2JointDef
  export class b2WheelJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2, axis: b2Vec2): void
    localAnchorA: b2Vec2
    localAnchorB: b2Vec2
    localAxisA: b2Vec2
    enableMotor: boolean
    maxMotorTorque: number
    motorSpeed: number
    frequencyHz: number
    dampingRatio: number
  }

  // b2MotorJoint implements b2Joint
  export class b2MotorJoint extends b2Joint {
    SetLinearOffset(linearOffset: b2Vec2): void
    GetLinearOffset(): b2Vec2
    SetAngularOffset(angularOffset: number): void
    GetAngularOffset(): number
    SetMaxForce(force: number): void
    GetMaxForce(): number
    SetMaxTorque(torque: number): void
    GetMaxTorque(): number
    SetCorrectionFactor(factor: number): void
    GetCorrectionFactor(): number

  }
  
  // b2MotorJointDef implements b2JointDef
  export class b2MotorJointDef extends b2JointDef {
    constructor()
    Initialize(bodyA: b2Body, bodyB: b2Body): void
    linearOffset: b2Vec2
    angularOffset: number
    maxForce: number
    maxTorque: number
    correctionFactor: number
  }

  interface Klass<T> {
    new(): T
  }
  // type Test<T extends keyof typeof Box2D> = {[P in T]: string}
  // let a: Box2D.b2Vec2
  export function wrapPointer<T>(ptr: any, cls: Klass<T>): T

}
// type Index = keyof typeof Box2D
// type ABox2D = { [k in Index]: typeof Box2D[k] }
// export interface IBox2D extends ABox2D {
//   [key: string]: any
// }



// export = Box2D
// export default Box2D