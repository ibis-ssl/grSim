/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pworld.h"

#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <typeinfo>

#if defined(HAVE_LINUX) || defined(HAVE_MACOSX)
#include <execinfo.h>
#endif

namespace {

void printBacktrace() {
#if defined(HAVE_LINUX) || defined(HAVE_MACOSX)
    void *frames[64];
    const int frameCount = backtrace(frames, 64);
    std::fputs("grSim: stack trace:\n", stderr);
    backtrace_symbols_fd(frames, frameCount, fileno(stderr));
#else
    std::fputs("grSim: stack trace is not supported on this platform.\n", stderr);
#endif
}

void printOdeMessage(const char *kind, int errnum, const char *msg, va_list ap) {
    std::fprintf(stderr, "grSim: ODE %s %d: ", kind, errnum);
    std::vfprintf(stderr, msg, ap);
    std::fputc('\n', stderr);
}

void odeErrorHandler(int errnum, const char *msg, va_list ap) {
    printOdeMessage("ERROR", errnum, msg, ap);
    printBacktrace();
    std::fflush(stderr);
    std::abort();
}

void odeDebugHandler(int errnum, const char *msg, va_list ap) {
    printOdeMessage("DEBUG", errnum, msg, ap);
    printBacktrace();
    std::fflush(stderr);
    std::abort();
}

void odeMessageHandler(int errnum, const char *msg, va_list ap) {
    printOdeMessage("MESSAGE", errnum, msg, ap);
    std::fflush(stderr);
}

void installOdeHandlers() {
    dSetErrorHandler(&odeErrorHandler);
    dSetDebugHandler(&odeDebugHandler);
    dSetMessageHandler(&odeMessageHandler);
}

bool isFiniteValue(dReal value) {
    return std::isfinite(static_cast<double>(value));
}

bool isFiniteVec3(const dReal *vec) {
    return isFiniteValue(vec[0]) && isFiniteValue(vec[1]) && isFiniteValue(vec[2]);
}

void dumpBodyState(const PObject *obj, dBodyID body) {
    const dReal *pos = dBodyGetPosition(body);
    const dReal *lin = dBodyGetLinearVel(body);
    const dReal *ang = dBodyGetAngularVel(body);
    const dReal *quat = dBodyGetQuaternion(body);

    std::fprintf(stderr,
                 "grSim: object %d (%s) body=%p\n"
                 "  pos=(%.9g, %.9g, %.9g)\n"
                 "  lin=(%.9g, %.9g, %.9g)\n"
                 "  ang=(%.9g, %.9g, %.9g)\n"
                 "  quat=(%.9g, %.9g, %.9g, %.9g)\n",
                 obj->id,
                 typeid(*obj).name(),
                 static_cast<void*>(body),
                 pos[0], pos[1], pos[2],
                 lin[0], lin[1], lin[2],
                 ang[0], ang[1], ang[2],
                 quat[0], quat[1], quat[2], quat[3]);
}

bool validateObjectState(const PObject *obj, unsigned long long stepIndex) {
    if (obj == nullptr || obj->geom == nullptr) {
        return true;
    }

    dReal aabb[6];
    dGeomGetAABB(obj->geom, aabb);
    const int geomClass = dGeomGetClass(obj->geom);
    const bool isInfinitePlane = (geomClass == dPlaneClass);
    const bool finiteAabb =
        isFiniteValue(aabb[0]) && isFiniteValue(aabb[1]) &&
        isFiniteValue(aabb[2]) && isFiniteValue(aabb[3]) &&
        isFiniteValue(aabb[4]) && isFiniteValue(aabb[5]);

    if (!finiteAabb && !isInfinitePlane) {
        std::fprintf(stderr,
                     "grSim: invalid AABB detected before collide at step=%llu, object=%d (%s)\n"
                     "  aabb=(%.9g, %.9g, %.9g, %.9g, %.9g, %.9g)\n",
                     stepIndex, obj->id, typeid(*obj).name(),
                     aabb[0], aabb[1], aabb[2], aabb[3], aabb[4], aabb[5]);
        const dBodyID body = dGeomGetBody(obj->geom);
        if (body != nullptr) {
            dumpBodyState(obj, body);
        }
        return false;
    }

    const dBodyID body = dGeomGetBody(obj->geom);
    if (body == nullptr) {
        return true;
    }

    const dReal *pos = dBodyGetPosition(body);
    const dReal *lin = dBodyGetLinearVel(body);
    const dReal *ang = dBodyGetAngularVel(body);
    const dReal *quat = dBodyGetQuaternion(body);

    const bool finiteState = isFiniteVec3(pos) && isFiniteVec3(lin) && isFiniteVec3(ang) &&
                             isFiniteValue(quat[0]) && isFiniteValue(quat[1]) &&
                             isFiniteValue(quat[2]) && isFiniteValue(quat[3]);
    if (!finiteState) {
        std::fprintf(stderr,
                     "grSim: invalid body state detected before collide at step=%llu, object=%d (%s)\n",
                     stepIndex, obj->id, typeid(*obj).name());
        dumpBodyState(obj, body);
        std::fprintf(stderr,
                     "  aabb=(%.9g, %.9g, %.9g, %.9g, %.9g, %.9g)\n",
                     aabb[0], aabb[1], aabb[2], aabb[3], aabb[4], aabb[5]);
        return false;
    }

    return true;
}

}  // namespace
PSurface::PSurface()
{
  callback = NULL;
  usefdir1 = false;
  surface.mode = dContactApprox1;
  surface.mu = 0.5;
}
bool PSurface::isIt(dGeomID i1,dGeomID i2)
{
    return ((i1==id1) && (i2==id2)) || ((i1==id2) && (i2==id1));
}


void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  ((PWorld*) data)->handleCollisions(o1,o2);
}


PWorld::PWorld(dReal dt,dReal gravity,CGraphics* graphics, int _robot_count)
{
    robot_count = _robot_count;
    installOdeHandlers();
    //dInitODE2(0);
    dInitODE();
    world = dWorldCreate();
    // Hash space quantizes AABB bounds and can assert on infinite bounds from plane geoms.
    // Use simple space to avoid integer-bound overflow with the ground plane.
    space = dSimpleSpaceCreate(0);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-gravity);
    objects_count = 0;
    sur_matrix = NULL;
    //dAllocateODEDataForThread(dAllocateMaskAll);
    delta_time = dt;
    g = graphics;
}

PWorld::~PWorld()
{
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
}

void PWorld::setGravity(dReal gravity)
{
    dWorldSetGravity (world,0,0,-gravity);
}

void PWorld::handleCollisions(dGeomID o1, dGeomID o2)
{   
    PSurface* sur;
    int j=sur_matrix[*((int*)(dGeomGetData(o1)))][*((int*)(dGeomGetData(o2)))];
    if (j!=-1)
    {
        const int N = 10;
        dContact contact[N];
        int n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
        if (n > 0) {
          sur = surfaces[j];
          sur->contactPos   [0] = contact[0].geom.pos[0];
          sur->contactPos   [1] = contact[0].geom.pos[1];
          sur->contactPos   [2] = contact[0].geom.pos[2];
          sur->contactNormal[0] = contact[0].geom.normal[0];
          sur->contactNormal[1] = contact[0].geom.normal[1];
          sur->contactNormal[2] = contact[0].geom.normal[2];
          bool flag=true;
          if (sur->callback!=NULL) flag = sur->callback(o1,o2,sur,robot_count);
          if (flag)
          for (int i=0; i<n; i++) {
              contact[i].surface = sur->surface;
              if (sur->usefdir1)
              {
                  contact[i].fdir1[0] = sur->fdir1[0];
                  contact[i].fdir1[1] = sur->fdir1[1];
                  contact[i].fdir1[2] = sur->fdir1[2];
                  contact[i].fdir1[3] = sur->fdir1[3];
              }
              dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);

              dJointAttach (c,
                            dGeomGetBody(contact[i].geom.g1),
                            dGeomGetBody(contact[i].geom.g2));
            }
        }
    }

}

void PWorld::addObject(PObject* o)
{      
    int id = objects.count();
    o->id = id;
    if (o->world==NULL) o->world = world;
    if (o->space==NULL) o->space = space;
    o->g = g;
    o->init();
    dGeomSetData(o->geom,(void*)(&(o->id)));
    objects.append(o);
}

void PWorld::initAllObjects()
{
    objects_count = objects.count();
    int c = objects_count;
    bool flag = false;
    if (sur_matrix!=NULL)
    {
        for (int i=0;i<c;i++)
            delete sur_matrix[i];
        delete sur_matrix;
        flag = true;
    }    
    sur_matrix = new int* [c];
    for (int i=0;i<c;i++)
    {
        sur_matrix[i] = new int [c];    
        for (int j=0;j<c;j++)
            sur_matrix[i][j] = -1;
    }
    if (flag)
    {
        for (int i=0;i<surfaces.count();i++)
            sur_matrix[(*(int*)(dGeomGetData(surfaces[i]->id1)))][*((int*)(dGeomGetData(surfaces[i]->id2)))] =
            sur_matrix[(*(int*)(dGeomGetData(surfaces[i]->id2)))][*((int*)(dGeomGetData(surfaces[i]->id1)))] = i;
    }
}

PSurface* PWorld::createSurface(PObject* o1,PObject* o2)
{
    PSurface *s = new PSurface();
    s->id1 = o1->geom;
    s->id2 = o2->geom;
    surfaces.append(s);
    sur_matrix[o1->id][o2->id] =
    sur_matrix[o2->id][o1->id] = surfaces.count() - 1;
    return s;
}

PSurface* PWorld::findSurface(PObject* o1,PObject* o2)
{
    for (int i=0;i<surfaces.count();i++)
    {
        if (surfaces[i]->isIt(o1->geom,o2->geom)) return (surfaces[i]);
    }
    return NULL;
}

void PWorld::step(dReal dt)
{
    try {
        static unsigned long long stepIndex = 0;
        ++stepIndex;
        for (int i = 0; i < objects.count(); ++i) {
            if (!validateObjectState(objects[i], stepIndex)) {
                std::fflush(stderr);
                std::abort();
            }
        }
        dSpaceCollide (space,this,&nearCallback);
        dWorldStep(world,(dt<0) ? delta_time : dt);
        dJointGroupEmpty (contactgroup);
    }
    catch (...) {
        //qDebug() << "Some Error Happened;";
    }
}

void PWorld::draw()
{
    for (int i=0;i<objects.count();i++)
        if (objects[i]->getVisibility()) objects[i]->draw();
}

void PWorld::glinit()
{
    for (int i=0;i<objects.count();i++)
        objects[i]->glinit();
}
