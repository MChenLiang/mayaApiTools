//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+
// DESCRIPTION:
// 
// Produces the MEL command "surfaceTwist".
//
// To use this command:
//  (1) First create and select a number of NURBS surfaces or polygons
//      (a fun surface to twist is the one created by the surfaceCreateCmd command).
//  (2) Enter the command "surfaceTwist" and all selected surfaces will
//      be twisted around the y-axis. 
//
// This command demonstrates how to access and modify the CVs of a NURBS surface
// or the vertices of a polygon. 
//


//-
// ==========================================================================
// Copyright 2015 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+
// DESCRIPTION:
//
// Produces the dependency graph node "yTwist".
//
// This plug-in demonstrates how to create a user-defined deformer.
// A deformer is a node which takes any number of input geometries, deforms
// them, and places the output into the output geometry attribute.
// This example plug-in defines a new deformer node that twists the deformed
// vertices of the input around the y-axis.
//
// To use this node: 
//  (1) Create a sphere or some other object. 
//  (2) Select the object. 
//  (3) Type: "deformer -type yTwist". 
//  (4) Bring up the channel box. 
//  (5) Select the yTwist input. 
//  (6) Change the Angle value of the yTwist input in the channel box. 


#include <math.h>
#include <maya/MIOStream.h>
#include <maya/MPxCommand.h>
#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MFnPlugin.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MDagPath.h>
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MItSurfaceCV.h>
#include <maya/MItMeshVertex.h>

#include <string.h>
#include <maya/MPxGeometryFilter.h>
#include <maya/MItGeometry.h>
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>

#define McheckErr(stat,msg)     \
    if ( MS::kSuccess != stat ) {   \
        cerr << msg;                \
        return MS::kFailure;        \
    }


// Class Definition //
class surfaceTwist : public MPxCommand {
public:
    surfaceTwist();
    virtual         ~surfaceTwist();
    virtual MStatus doIt(const MArgList& args);
    static void*    creator();
};
// Class Implementation //
#define NUM_SPANS        30
#define WIDTH            10.0
#define VERTICAL_SCALING 4.0
surfaceTwist::~surfaceTwist() {}
surfaceTwist::surfaceTwist() {}
void* surfaceTwist::creator()
{
    return new surfaceTwist;
}
static MStatus twistNurbsSurface(MDagPath& objectPath, MObject& component)
{
    MStatus status;
    MPoint  center;
    MVector toCenter(-center.x, 0.0, -center.y);
    double  rotFactor = 0.5;
    // We have a nurbs surface or component
    //
    MItSurfaceCV cvIter(objectPath, component, true, &status);
    if (MS::kSuccess == status) {
        // We successfully created a nurbs surface iterator
        //
        for (; !cvIter.isDone(); cvIter.nextRow()) {
            for (; !cvIter.isRowDone(); cvIter.next()) {
                // Get the location of the CV
                //
                MPoint pnt = cvIter.position(MSpace::kWorld);
                pnt = pnt + toCenter;
                // Calculate rotation in radians about the y-axis
                //
                double rotation = pnt.y * rotFactor;
                MMatrix rotMatrix;
                // Set matrix to a rotation about the y axis
                //
                rotMatrix(0, 0) = cos(rotation);
                rotMatrix(0, 2) = sin(rotation);
                rotMatrix(2, 0) = -sin(rotation);
                rotMatrix(2, 2) = cos(rotation);
                pnt = (pnt * rotMatrix) - toCenter;
                status = cvIter.setPosition(pnt, MSpace::kWorld);
                if (MS::kSuccess != status)
                    break;
            }
        }
        // Tell maya to redraw the surface with all of our changes
        //
        cvIter.updateSurface();
        return MS::kSuccess;
    }
    else
        return MS::kFailure;
}
static MStatus twistPolygon(MDagPath& objectPath, MObject& component)
{
    MStatus status;
    MPoint  center;
    MVector toCenter(-center.x, 0.0, -center.y);
    double  rotFactor = 0.5;
    MItMeshVertex vertIter(objectPath, component, &status);
    if (MS::kSuccess == status) {
        // We successfully created a polygon vertex iterator
        //
        for (; !vertIter.isDone(); vertIter.next()) {
            // Get the location of the vertex
            //
            MPoint pnt = vertIter.position(MSpace::kWorld);
            pnt = pnt + toCenter;
            // Calculate rotation in radians about the y-axis
            //
            double rotation = pnt.y * rotFactor;
            MMatrix rotMatrix;
            // Set matrix to a rotation about the y axis
            //
            rotMatrix(0, 0) = cos(rotation);
            rotMatrix(0, 2) = sin(rotation);
            rotMatrix(2, 0) = -sin(rotation);
            rotMatrix(2, 2) = cos(rotation);
            pnt = (pnt * rotMatrix) - toCenter;
            status = vertIter.setPosition(pnt, MSpace::kWorld);
            if (MS::kSuccess != status)
                break;
        }
        // Tell maya to redraw the surface with all of our changes
        //
        vertIter.updateSurface();
        return MS::kSuccess;
    }
    else
        return MS::kFailure;
}
MStatus surfaceTwist::doIt(const MArgList&)
//
//  Description:
//      Plugin command to test Selection List Iterator.
//
//
{
    MStatus status;
    // Create an iterator for the active selection list
    //
    MSelectionList slist;
    MGlobal::getActiveSelectionList(slist);
    MItSelectionList iter(slist);
    if (iter.isDone()) {
        cerr << "Nothing selected\n";
        return MS::kFailure;
    }
    MDagPath objectPath;
    MObject component;
    for (; !iter.isDone(); iter.next()) {
        status = iter.getDagPath(objectPath, component);
        if (objectPath.hasFn(MFn::kNurbsSurface))
            status = twistNurbsSurface(objectPath, component);
        else if (objectPath.hasFn(MFn::kMesh))
            status = twistPolygon(objectPath, component);
        else {
            cerr << "Selected object is not a NURBS surface or a polygon\n";
            return MS::kFailure;
        }
    }
    return status;
}


// y twist
class yTwist : public MPxGeometryFilter
{
public:
    yTwist();
    virtual             ~yTwist();
    static  void*       creator();
    static  MStatus     initialize();
    // deformation function
    //
    virtual MStatus     deform(MDataBlock&      block,
        MItGeometry&     iter,
        const MMatrix&   mat,
        unsigned int     multiIndex);
public:
    // yTwist attributes
    //
    static  MObject     angle;          // angle to twist

    static  MTypeId     id;
private:
};
MTypeId     yTwist::id(0x8000e);
// yTwist attributes  //
MObject     yTwist::angle;
yTwist::yTwist()
//
//  Description:
//      constructor
//
{
}
yTwist::~yTwist()
//
//  Description:
//      destructor
//
{}
void* yTwist::creator()
//
//  Description:
//      create the yTwist
//
{
    return new yTwist();
}
MStatus yTwist::initialize()
//
//  Description:
//      initialize the attributes
//
{
    // local attribute initialization
    //
    MFnNumericAttribute nAttr;
    angle = nAttr.create("angle", "fa", MFnNumericData::kDouble);
    nAttr.setDefault(0.0);
    nAttr.setKeyable(true);
    addAttribute(angle);
    // affects
    //
    attributeAffects(yTwist::angle, yTwist::outputGeom);
    return MS::kSuccess;
}
MStatus
yTwist::deform(MDataBlock& block,
    MItGeometry& iter,
    const MMatrix& /*m*/,
    unsigned int /*multiIndex*/)
    //
    // Method: deform
    //
    // Description:   Deform the point with a yTwist algorithm
    //
    // Arguments:
    //   block      : the datablock of the node
    //   iter       : an iterator for the geometry to be deformed
    //   m          : matrix to transform the point into world space
    //   multiIndex : the index of the geometry that we are deforming
    //
    //
{
    MStatus status = MS::kSuccess;

    // determine the angle of the yTwist
    //
    MDataHandle angleData = block.inputValue(angle, &status);
    McheckErr(status, "Error getting angle data handle\n");
    double magnitude = angleData.asDouble();
    // determine the envelope (this is a global scale factor)
    //
    MDataHandle envData = block.inputValue(envelope, &status);
    McheckErr(status, "Error getting envelope data handle\n");
    float env = envData.asFloat();
    // iterate through each point in the geometry
    //
    for (; !iter.isDone(); iter.next()) {

        MPoint pt = iter.position();
        // do the twist
        //
        double ff = magnitude * pt.y*env;
        if (ff != 0.0) {
            double cct = cos(ff);
            double cst = sin(ff);
            double tt = pt.x*cct - pt.z*cst;
            pt.z = pt.x*cst + pt.z*cct;
            pt.x = tt;;
        }
        iter.setPosition(pt);
    }
    return status;
}
// standard initialization procedures
//



// Register command with system //
MStatus initializePlugin(MObject obj)
{
    MStatus   status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "Any");
    status = plugin.registerCommand("surfaceTwist", surfaceTwist::creator);
    if (!status) {
        status.perror("registerCommand");
        return status;
    }
    status = plugin.registerNode("yTwist", yTwist::id, yTwist::creator,
        yTwist::initialize, MPxNode::kDeformerNode);
    if (!status) {
        status.perror("registerCommand");
        return status;
    }
    return status;
}
MStatus uninitializePlugin(MObject obj)
{
    MStatus   status;
    MFnPlugin plugin(obj);
    status = plugin.deregisterCommand("surfaceTwist");
    if (!status) {
        status.perror("deregisterCommand");
        return status;
    }
    status = plugin.deregisterNode(yTwist::id);
    if (!status) {
        status.perror("deregisterCommand");
        return status;
    }
    return status;
}
