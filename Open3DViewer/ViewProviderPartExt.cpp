/***************************************************************************
 *   Copyright (c) 2011 Juergen Riegel <juergen.riegel@web.de>             *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoPolygonOffset.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/details/SoFaceDetail.h>
#include <Inventor/details/SoLineDetail.h>
#include <Poly_Connect.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <Precision.hxx>
#include <GeomLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <QColor>
#include <Open3DViewer\SoFCSelectionAction.h>
#include <Open3DViewer\SoFCUnifiedSelection.h>
#include <Open3DViewer\ViewProviderPartExt.h>

ViewProviderPartExt::ViewProviderPartExt()
{
	NormalsFromUV = true;
	LineWidth = 1;
	PointSize = 1;
	Deviation = double(0.2);
	AngularDeflection = 28.65;


	coords = new SoCoordinate3();
	coords->ref();
	faceset = new SoBrepFaceSet();
	faceset->ref();
	norm = new SoNormal;
	norm->ref();
	normb = new SoNormalBinding;
	normb->value = SoNormalBinding::PER_VERTEX_INDEXED;
	normb->ref();

	lineset = new SoBrepEdgeSet();
	lineset->ref();

	nodeset = new SoBrepPointSet();
	nodeset->ref();

	pcShapeMaterial = new SoMaterial;
	pcShapeMaterial->ref();

	pcFaceBind = new SoMaterialBinding();
	pcFaceBind->ref();

	pcLineBind = new SoMaterialBinding();
	pcLineBind->ref();
	pcLineMaterial = new SoMaterial;
	pcLineMaterial->ref();
	pcLineMaterial->ambientColor.setValue(0.2, 0.2, 0.2);
	pcLineMaterial->diffuseColor.setValue(0.1,0.1,0.1);
	pcLineMaterial->specularColor.setValue(0,0,0);
	pcLineMaterial->emissiveColor.setValue(0,0,0);
	pcLineMaterial->shininess.setValue(1);
	pcLineMaterial->transparency.setValue(0);



	pcPointBind = new SoMaterialBinding();
	pcPointBind->ref();
	pcPointMaterial = new SoMaterial;
	pcPointMaterial->ref();
	pcPointMaterial->ambientColor.setValue(0.2, 0.2, 0.2);
	pcPointMaterial->diffuseColor.setValue(0.1, 0.1, 0.1);
	pcPointMaterial->specularColor.setValue(0, 0, 0);
	pcPointMaterial->emissiveColor.setValue(0, 0, 0);
	pcPointMaterial->shininess.setValue(1);
	pcPointMaterial->transparency.setValue(0);

	

	


	pcLineStyle = new SoDrawStyle();
	pcLineStyle->ref();
	pcLineStyle->style = SoDrawStyle::LINES;
	pcLineStyle->lineWidth = LineWidth;
	pcLineStyle->linePattern = 0xffff;

	pcPointStyle = new SoDrawStyle();
	pcPointStyle->ref();
	pcPointStyle->style = SoDrawStyle::POINTS;
	pcPointStyle->pointSize = PointSize;

	pShapeHints = new SoShapeHints;
	pShapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
	pShapeHints->ref();
	pShapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;

	




	// Workaround for #0000433, i.e. use SoSeparator instead of SoGroup
	auto* pcNormalRoot = new SoSeparator();
	auto* pcFlatRoot = new SoSeparator();
	auto* pcWireframeRoot = new SoSeparator();
	auto* pcPointsRoot = new SoSeparator();
	auto* wireframe = new SoSeparator();

	// Must turn off all intermediate render caching, and let pcRoot to handle
	// cache without interference.
	pcNormalRoot->renderCaching =
		pcFlatRoot->renderCaching =
		pcWireframeRoot->renderCaching =
		pcPointsRoot->renderCaching =
		wireframe->renderCaching = SoSeparator::OFF;

	pcNormalRoot->boundingBoxCaching =
		pcFlatRoot->boundingBoxCaching =
		pcWireframeRoot->boundingBoxCaching =
		pcPointsRoot->boundingBoxCaching =
		wireframe->boundingBoxCaching = SoSeparator::OFF;

	// Avoid any Z-buffer artifacts, so that the lines always appear on top of the faces
	// The correct order is Edges, Polygon offset, Faces.
	SoPolygonOffset* offset = new SoPolygonOffset();

	// wireframe node
	wireframe->setName("Edge");
	wireframe->addChild(pcLineBind);
	wireframe->addChild(pcLineMaterial);
	wireframe->addChild(pcLineStyle);
	wireframe->addChild(lineset);

	// normal viewing with edges and points
	pcNormalRoot->addChild(pcPointsRoot);
	pcNormalRoot->addChild(wireframe);
	pcNormalRoot->addChild(offset);
	pcNormalRoot->addChild(pcFlatRoot);

	// just faces with no edges or points
	pcFlatRoot->addChild(pShapeHints);
	pcFlatRoot->addChild(pcFaceBind);
	pcFlatRoot->addChild(pcShapeMaterial);
	SoDrawStyle* pcFaceStyle = new SoDrawStyle();
	pcFaceStyle->style = SoDrawStyle::FILLED;
	pcFlatRoot->addChild(pcFaceStyle);
	pcFlatRoot->addChild(norm);
	pcFlatRoot->addChild(normb);
	pcFlatRoot->addChild(faceset);

	// edges and points
	pcWireframeRoot->addChild(wireframe);
	pcWireframeRoot->addChild(pcPointsRoot);

	// normal viewing with edges and points
	pcPointsRoot->addChild(pcPointBind);
	pcPointsRoot->addChild(pcPointMaterial);
	pcPointsRoot->addChild(pcPointStyle);
	pcPointsRoot->addChild(nodeset);

	// Move 'coords' before the switch
	pcRoot->insertChild(coords, pcRoot->findChild(pcModeSwitch));
	//pcRoot->addChild(coords);

	pcModeSwitch->addChild(pcNormalRoot);
	pcModeSwitch->addChild(pcFlatRoot);
	pcModeSwitch->addChild(pcWireframeRoot);
	pcModeSwitch->addChild(pcPointsRoot);

	addDisplayMaskMode(pcNormalRoot, "Flat Lines");
	addDisplayMaskMode(pcFlatRoot, "Shaded");
	addDisplayMaskMode(pcWireframeRoot, "Wireframe");
	addDisplayMaskMode(pcPointsRoot, "Point");
}

void ViewProviderPartExt::setShape(TopoDS_Shape shape)
{
	if (shape.IsNull()) return;
	_shape = shape;
	updateVisual();
	pcModeSwitch->whichChild = 0;
	//newUpdateVisual();
	this->pcFaceBind->value = SoMaterialBinding::OVERALL;
	
}

TopoDS_Shape ViewProviderPartExt::getShape() const
{
	return _shape;
}

void ViewProviderPartExt::getNormals(const TopoDS_Face&  theFace,
	const Handle(Poly_Triangulation)& aPolyTri,
	TColgp_Array1OfDir& theNormals)
{
	const TColgp_Array1OfPnt& aNodes = aPolyTri->Nodes();

	if (aPolyTri->HasNormals())
	{
		// normals pre-computed in triangulation structure
		const TShort_Array1OfShortReal& aNormals = aPolyTri->Normals();
		const Standard_ShortReal*       aNormArr = &(aNormals.Value(aNormals.Lower()));

		for (Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
		{
			const Standard_Integer anId = 3 * (aNodeIter - aNodes.Lower());
			const gp_Dir aNorm(aNormArr[anId + 0],
				aNormArr[anId + 1],
				aNormArr[anId + 2]);
			theNormals(aNodeIter) = aNorm;
		}

		if (theFace.Orientation() == TopAbs_REVERSED)
		{
			for (Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
			{
				theNormals.ChangeValue(aNodeIter).Reverse();
			}
		}

		return;
	}

	// take in face the surface location
	Poly_Connect thePolyConnect(aPolyTri);
	const TopoDS_Face      aZeroFace = TopoDS::Face(theFace.Located(TopLoc_Location()));
	Handle(Geom_Surface)   aSurf = BRep_Tool::Surface(aZeroFace);
	const Standard_Real    aTol = Precision::Confusion();
	Handle(TShort_HArray1OfShortReal) aNormals = new TShort_HArray1OfShortReal(1, aPolyTri->NbNodes() * 3);
	const Poly_Array1OfTriangle& aTriangles = aPolyTri->Triangles();
	const TColgp_Array1OfPnt2d*  aNodesUV = aPolyTri->HasUVNodes() && !aSurf.IsNull()
		? &aPolyTri->UVNodes()
		: NULL;
	Standard_Integer aTri[3];

	for (Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
	{
		// try to retrieve normal from real surface first, when UV coordinates are available
		if (aNodesUV == NULL
			|| GeomLib::NormEstim(aSurf, aNodesUV->Value(aNodeIter), aTol, theNormals(aNodeIter)) > 1)
		{
			// compute flat normals
			gp_XYZ eqPlan(0.0, 0.0, 0.0);

			for (thePolyConnect.Initialize(aNodeIter); thePolyConnect.More(); thePolyConnect.Next())
			{
				aTriangles(thePolyConnect.Value()).Get(aTri[0], aTri[1], aTri[2]);
				const gp_XYZ v1(aNodes(aTri[1]).Coord() - aNodes(aTri[0]).Coord());
				const gp_XYZ v2(aNodes(aTri[2]).Coord() - aNodes(aTri[1]).Coord());
				const gp_XYZ vv = v1 ^ v2;
				const Standard_Real aMod = vv.Modulus();

				if (aMod >= aTol)
				{
					eqPlan += vv / aMod;
				}
			}

			const Standard_Real aModMax = eqPlan.Modulus();
			theNormals(aNodeIter) = (aModMax > aTol) ? gp_Dir(eqPlan) : gp::DZ();
		}

		const Standard_Integer anId = (aNodeIter - aNodes.Lower()) * 3;
		aNormals->SetValue(anId + 1, (Standard_ShortReal)theNormals(aNodeIter).X());
		aNormals->SetValue(anId + 2, (Standard_ShortReal)theNormals(aNodeIter).Y());
		aNormals->SetValue(anId + 3, (Standard_ShortReal)theNormals(aNodeIter).Z());
	}

	aPolyTri->SetNormals(aNormals);

	if (theFace.Orientation() == TopAbs_REVERSED)
	{
		for (Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
		{
			theNormals.ChangeValue(aNodeIter).Reverse();
		}
	}
}

void ViewProviderPartExt::updateVisual()
{
	SoUpdateVBOAction action;
	action.apply(this->faceset);

	// Clear selection
	SoSelectionElementAction saction(SoSelectionElementAction::None);
	saction.apply(this->faceset);
	saction.apply(this->lineset);
	saction.apply(this->nodeset);

	// Clear highlighting
	SoHighlightElementAction haction;
	haction.apply(this->faceset);
	haction.apply(this->lineset);
	haction.apply(this->nodeset);

	TopoDS_Shape cShape = _shape;
	if (cShape.IsNull()) {
		coords->point.setNum(0);
		norm->vector.setNum(0);
		faceset->coordIndex.setNum(0);
		faceset->partIndex.setNum(0);
		lineset->coordIndex.setNum(0);
		nodeset->startIndex.setValue(0);
		VisualTouched = false;
		return;
	}

	// time measurement and book keeping
	//Base::TimeInfo start_time;
	int numTriangles = 0, numNodes = 0, numNorms = 0, numFaces = 0, numEdges = 0, numLines = 0;
	std::set<int> faceEdges;

	try {
		// calculating the deflection value
		Bnd_Box bounds;
		BRepBndLib::Add(cShape, bounds);
		bounds.SetGap(0.0);
		Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
		bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
		Standard_Real deflection = ((xMax - xMin) + (yMax - yMin) + (zMax - zMin)) / 300.0 *
			Deviation;

		// create or use the mesh on the data structure
#if OCC_VERSION_HEX >= 0x060600
		Standard_Real AngDeflectionRads = AngularDeflection / 180.0 * M_PI;
		BRepMesh_IncrementalMesh(cShape, deflection, Standard_False,
			AngDeflectionRads, Standard_True);
#else
		BRepMesh_IncrementalMesh(cShape, deflection);
#endif
		// We must reset the location here because the transformation data
		// are set in the placement property
		TopLoc_Location aLoc;
		cShape.Location(aLoc);

		// count triangles and nodes in the mesh
		TopTools_IndexedMapOfShape faceMap;
		TopExp::MapShapes(cShape, TopAbs_FACE, faceMap);
		for (int i = 1; i <= faceMap.Extent(); i++) {
			Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(TopoDS::Face(faceMap(i)), aLoc);
			// Note: we must also count empty faces
			if (!mesh.IsNull()) {
				numTriangles += mesh->NbTriangles();
				numNodes += mesh->NbNodes();
				numNorms += mesh->NbNodes();
			}

			TopExp_Explorer xp;
			for (xp.Init(faceMap(i), TopAbs_EDGE); xp.More(); xp.Next())
				faceEdges.insert(xp.Current().HashCode(INT_MAX));
			numFaces++;
		}

		// get an indexed map of edges
		TopTools_IndexedMapOfShape edgeMap;
		TopExp::MapShapes(cShape, TopAbs_EDGE, edgeMap);

		// key is the edge number, value the coord indexes. This is needed to keep the same order as the edges.
		std::map<int, std::vector<int32_t> > lineSetMap;
		std::set<int>          edgeIdxSet;
		std::vector<int32_t>   edgeVector;

		// count and index the edges
		for (int i = 1; i <= edgeMap.Extent(); i++) {
			edgeIdxSet.insert(i);
			numEdges++;

			const TopoDS_Edge& aEdge = TopoDS::Edge(edgeMap(i));
			TopLoc_Location aLoc;

			// handling of the free edge that are not associated to a face
			// Note: The assumption that if for an edge BRep_Tool::Polygon3D
			// returns a valid object is wrong. This e.g. happens for ruled
			// surfaces which gets created by two edges or wires.
			// So, we have to store the hashes of the edges associated to a face.
			// If the hash of a given edge is not in this list we know it's really
			// a free edge.
			int hash = aEdge.HashCode(INT_MAX);
			if (faceEdges.find(hash) == faceEdges.end()) {
				Handle(Poly_Polygon3D) aPoly = BRep_Tool::Polygon3D(aEdge, aLoc);
				if (!aPoly.IsNull()) {
					int nbNodesInEdge = aPoly->NbNodes();
					numNodes += nbNodesInEdge;
				}
			}
		}

		// handling of the vertices
		TopTools_IndexedMapOfShape vertexMap;
		TopExp::MapShapes(cShape, TopAbs_VERTEX, vertexMap);
		numNodes += vertexMap.Extent();

		// create memory for the nodes and indexes
		coords->point.setNum(numNodes);
		norm->vector.setNum(numNorms);
		faceset->coordIndex.setNum(numTriangles * 4);
		faceset->partIndex.setNum(numFaces);
		// get the raw memory for fast fill up
		SbVec3f* verts = coords->point.startEditing();
		SbVec3f* norms = norm->vector.startEditing();
		int32_t* index = faceset->coordIndex.startEditing();
		int32_t* parts = faceset->partIndex.startEditing();

		// preset the normal vector with null vector
		for (int i = 0; i < numNorms; i++)
			norms[i] = SbVec3f(0.0, 0.0, 0.0);

		int ii = 0, faceNodeOffset = 0, faceTriaOffset = 0;
		for (int i = 1; i <= faceMap.Extent(); i++, ii++) {
			TopLoc_Location aLoc;
			const TopoDS_Face &actFace = TopoDS::Face(faceMap(i));
			// get the mesh of the shape
			Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(actFace, aLoc);
			if (mesh.IsNull()) {
				parts[ii] = 0;
				continue;
			}

			// getting the transformation of the shape/face
			gp_Trsf myTransf;
			Standard_Boolean identity = true;
			if (!aLoc.IsIdentity()) {
				identity = false;
				myTransf = aLoc.Transformation();
			}

			// getting size of node and triangle array of this face
			int nbNodesInFace = mesh->NbNodes();
			int nbTriInFace = mesh->NbTriangles();
			// check orientation
			TopAbs_Orientation orient = actFace.Orientation();


			// cycling through the poly mesh
			const Poly_Array1OfTriangle& Triangles = mesh->Triangles();
			const TColgp_Array1OfPnt& Nodes = mesh->Nodes();
			TColgp_Array1OfDir Normals(Nodes.Lower(), Nodes.Upper());
			if (NormalsFromUV)
				getNormals(actFace, mesh, Normals);

			for (int g = 1; g <= nbTriInFace; g++) {
				// Get the triangle
				Standard_Integer N1, N2, N3;
				Triangles(g).Get(N1, N2, N3);

				// change orientation of the triangle if the face is reversed
				if (orient != TopAbs_FORWARD) {
					Standard_Integer tmp = N1;
					N1 = N2;
					N2 = tmp;
				}

				// get the 3 points of this triangle
				gp_Pnt V1(Nodes(N1)), V2(Nodes(N2)), V3(Nodes(N3));

				// get the 3 normals of this triangle
				gp_Vec NV1, NV2, NV3;
				if (NormalsFromUV) {
					NV1.SetXYZ(Normals(N1).XYZ());
					NV2.SetXYZ(Normals(N2).XYZ());
					NV3.SetXYZ(Normals(N3).XYZ());
				}
				else {
					gp_Vec v1(V1.X(), V1.Y(), V1.Z()),
						v2(V2.X(), V2.Y(), V2.Z()),
						v3(V3.X(), V3.Y(), V3.Z());
					gp_Vec normal = (v2 - v1) ^ (v3 - v1);
					NV1 = normal;
					NV2 = normal;
					NV3 = normal;
				}

				// transform the vertices and normals to the place of the face
				if (!identity) {
					V1.Transform(myTransf);
					V2.Transform(myTransf);
					V3.Transform(myTransf);
					if (NormalsFromUV) {
						NV1.Transform(myTransf);
						NV2.Transform(myTransf);
						NV3.Transform(myTransf);
					}
				}

				// add the normals for all points of this triangle
				norms[faceNodeOffset + N1 - 1] += SbVec3f(NV1.X(), NV1.Y(), NV1.Z());
				norms[faceNodeOffset + N2 - 1] += SbVec3f(NV2.X(), NV2.Y(), NV2.Z());
				norms[faceNodeOffset + N3 - 1] += SbVec3f(NV3.X(), NV3.Y(), NV3.Z());

				// set the vertices
				verts[faceNodeOffset + N1 - 1].setValue((float)(V1.X()), (float)(V1.Y()), (float)(V1.Z()));
				verts[faceNodeOffset + N2 - 1].setValue((float)(V2.X()), (float)(V2.Y()), (float)(V2.Z()));
				verts[faceNodeOffset + N3 - 1].setValue((float)(V3.X()), (float)(V3.Y()), (float)(V3.Z()));

				// set the index vector with the 3 point indexes and the end delimiter
				index[faceTriaOffset * 4 + 4 * (g - 1)] = faceNodeOffset + N1 - 1;
				index[faceTriaOffset * 4 + 4 * (g - 1) + 1] = faceNodeOffset + N2 - 1;
				index[faceTriaOffset * 4 + 4 * (g - 1) + 2] = faceNodeOffset + N3 - 1;
				index[faceTriaOffset * 4 + 4 * (g - 1) + 3] = SO_END_FACE_INDEX;
			}

			parts[ii] = nbTriInFace; // new part

			// handling the edges lying on this face
			TopExp_Explorer Exp;
			for (Exp.Init(actFace, TopAbs_EDGE); Exp.More(); Exp.Next()) {
				const TopoDS_Edge &curEdge = TopoDS::Edge(Exp.Current());
				// get the overall index of this edge
				int edgeIndex = edgeMap.FindIndex(curEdge);
				edgeVector.push_back((int32_t)edgeIndex - 1);
				// already processed this index ?
				if (edgeIdxSet.find(edgeIndex) != edgeIdxSet.end()) {

					// this holds the indices of the edge's triangulation to the current polygon
					Handle(Poly_PolygonOnTriangulation) aPoly = BRep_Tool::PolygonOnTriangulation(curEdge, mesh, aLoc);
					if (aPoly.IsNull())
						continue; // polygon does not exist

					// getting the indexes of the edge polygon
					const TColStd_Array1OfInteger& indices = aPoly->Nodes();
					for (Standard_Integer i = indices.Lower(); i <= indices.Upper(); i++) {
						int nodeIndex = indices(i);
						int index = faceNodeOffset + nodeIndex - 1;
						lineSetMap[edgeIndex].push_back(index);

						// usually the coordinates for this edge are already set by the
						// triangles of the face this edge belongs to. However, there are
						// rare cases where some points are only referenced by the polygon
						// but not by any triangle. Thus, we must apply the coordinates to
						// make sure that everything is properly set.
						gp_Pnt p(Nodes(nodeIndex));
						if (!identity)
							p.Transform(myTransf);
						verts[index].setValue((float)(p.X()), (float)(p.Y()), (float)(p.Z()));
					}

					// remove the handled edge index from the set
					edgeIdxSet.erase(edgeIndex);
				}
			}

			edgeVector.push_back(-1);

			// counting up the per Face offsets
			faceNodeOffset += nbNodesInFace;
			faceTriaOffset += nbTriInFace;
		}

		// handling of the free edges
		for (int i = 1; i <= edgeMap.Extent(); i++) {
			const TopoDS_Edge& aEdge = TopoDS::Edge(edgeMap(i));
			Standard_Boolean identity = true;
			gp_Trsf myTransf;
			TopLoc_Location aLoc;

			// handling of the free edge that are not associated to a face
			int hash = aEdge.HashCode(INT_MAX);
			if (faceEdges.find(hash) == faceEdges.end()) {
				Handle(Poly_Polygon3D) aPoly = BRep_Tool::Polygon3D(aEdge, aLoc);
				if (!aPoly.IsNull()) {
					if (!aLoc.IsIdentity()) {
						identity = false;
						myTransf = aLoc.Transformation();
					}

					const TColgp_Array1OfPnt& aNodes = aPoly->Nodes();
					int nbNodesInEdge = aPoly->NbNodes();

					gp_Pnt pnt;
					for (Standard_Integer j = 1; j <= nbNodesInEdge; j++) {
						pnt = aNodes(j);
						if (!identity)
							pnt.Transform(myTransf);
						int index = faceNodeOffset + j - 1;
						verts[index].setValue((float)(pnt.X()), (float)(pnt.Y()), (float)(pnt.Z()));
						lineSetMap[i].push_back(index);
					}

					faceNodeOffset += nbNodesInEdge;
				}
			}
		}

		nodeset->startIndex.setValue(faceNodeOffset);
		for (int i = 0; i < vertexMap.Extent(); i++) {
			const TopoDS_Vertex& aVertex = TopoDS::Vertex(vertexMap(i + 1));
			gp_Pnt pnt = BRep_Tool::Pnt(aVertex);
			verts[faceNodeOffset + i].setValue((float)(pnt.X()), (float)(pnt.Y()), (float)(pnt.Z()));
		}

		// normalize all normals 
		for (int i = 0; i < numNorms; i++)
			norms[i].normalize();

		std::vector<int32_t> lineSetCoords;
		for (std::map<int, std::vector<int32_t> >::iterator it = lineSetMap.begin(); it != lineSetMap.end(); ++it) {
			lineSetCoords.insert(lineSetCoords.end(), it->second.begin(), it->second.end());
			lineSetCoords.push_back(-1);
		}

		// preset the index vector size
		numLines = lineSetCoords.size();
		lineset->coordIndex.setNum(numLines);
		int32_t* lines = lineset->coordIndex.startEditing();

		int l = 0;
		for (std::vector<int32_t>::const_iterator it = lineSetCoords.begin(); it != lineSetCoords.end(); ++it, l++)
			lines[l] = *it;

		// end the editing of the nodes
		coords->point.finishEditing();
		norm->vector.finishEditing();
		faceset->coordIndex.finishEditing();
		faceset->partIndex.finishEditing();
		lineset->coordIndex.finishEditing();
	}
	catch (...) {
		//FC_ERR("Cannot compute Inventor representation for the shape of " << pcObject->getFullName());
	}

#   ifdef FC_DEBUG
	// printing some information
	Base::Console().Log("ViewProvider update time: %f s\n", Base::TimeInfo::diffTimeF(start_time, Base::TimeInfo()));
	Base::Console().Log("Shape tria info: Faces:%d Edges:%d Nodes:%d Triangles:%d IdxVec:%d\n", numFaces, numEdges, numNodes, numTriangles, numLines);
#   endif
	VisualTouched = false;
}


std::string ViewProviderPartExt::getElement(const SoDetail* detail) const
{
	std::stringstream str;
	if (detail) {
		if (detail->getTypeId() == SoFaceDetail::getClassTypeId()) {
			const SoFaceDetail* face_detail = static_cast<const SoFaceDetail*>(detail);
			int face = face_detail->getPartIndex() + 1;
			str << "Face" << face;
		}
		else if (detail->getTypeId() == SoLineDetail::getClassTypeId()) {
			const SoLineDetail* line_detail = static_cast<const SoLineDetail*>(detail);
			int edge = line_detail->getLineIndex() + 1;
			str << "Edge" << edge;
		}
		else if (detail->getTypeId() == SoPointDetail::getClassTypeId()) {
			const SoPointDetail* point_detail = static_cast<const SoPointDetail*>(detail);
			int vertex = point_detail->getCoordinateIndex() - nodeset->startIndex.getValue() + 1;
			str << "Vertex" << vertex;
		}
	}

	return str.str();
}

bool ViewProviderPartExt::getElementPicked(const SoPickedPoint *pp, std::string &subname) const
{
	if (!isSelectable()) return false;
	subname = getElement(pp ? pp->getDetail() : 0);
	return true;
	//auto childRoot = getChildRoot();
	//int idx;
	//if (!childRoot ||
	//	(idx = pcModeSwitch->whichChild.getValue()) < 0 ||
	//	pcModeSwitch->getChild(idx) != childRoot)
	//{
	//	return ViewProvider::getElementPicked(pp, subname);
	//}

	//SoPath* path = pp->getPath();
	//idx = path->findNode(childRoot);
	//if (idx < 0 || idx + 1 >= path->getLength())
	//	return false;
	//auto vp = getDocument()->getViewProvider(path->getNode(idx + 1));
	//if (!vp) return false;
	//auto obj = vp->getObject();
	//if (!obj || !obj->getNameInDocument())
	//	return false;
	//std::ostringstream str;
	//str << obj->getNameInDocument() << '.';
	//if (vp->getElementPicked(pp, subname))
	//	str << subname;
	//subname = str.str();
	//return true;
}

SoDetail* ViewProviderPartExt::getDetail(const char* subelement) const
{
	std::string element = subelement;
	std::string::size_type pos = element.find_first_of("0123456789");
	int index = -1;
	if (pos != std::string::npos) {
		index = std::atoi(element.substr(pos).c_str());
		element = element.substr(0, pos);
	}

	SoDetail* detail = 0;
	if (index < 0)
		return detail;
	if (element == "Face") {
		detail = new SoFaceDetail();
		static_cast<SoFaceDetail*>(detail)->setPartIndex(index - 1);
	}
	else if (element == "Edge") {
		detail = new SoLineDetail();
		static_cast<SoLineDetail*>(detail)->setLineIndex(index - 1);
	}
	else if (element == "Vertex") {
		detail = new SoPointDetail();
		static_cast<SoPointDetail*>(detail)->setCoordinateIndex(index + nodeset->startIndex.getValue() - 1);
	}

	return detail;
}

void ViewProviderPartExt::setShapeColor(double r, double g, double b)
{
	pcShapeMaterial->diffuseColor.setValue(r / 256.0, g / 256.0, b / 256.0);
	DiffuseColor.clear();
	DiffuseColor.emplace_back(App::Color(r / 256.0, g / 256.0, b / 256.0));
	globeColor.set(r / 256.0, g / 256.0, b / 256.0);
}

void ViewProviderPartExt::applyColors(std::vector<Color> colors)
{
	if (colors.empty()) {
		// vp->MapFaceColor.setValue(true);
		// vp->MapLineColor.setValue(true);
		// vp->updateColors(0,true);
		return;
	}
	// vp->MapFaceColor.setValue(false);
	//if (colors.size() == 1)
	//	ShapeColor.setValue(colors.front());
	//else
	//	DiffuseColor.setValues(colors);
	setHighlightedFaces(colors);
}

void ViewProviderPartExt::setHighlightedFaces(const std::vector<App::Color>& colors)
{
	DiffuseColor = colors;
	SoUpdateVBOAction action;
	action.apply(this->faceset);

	int size = static_cast<int>(colors.size());
	if (size > 1 && size == this->faceset->partIndex.getNum()) {
		pcFaceBind->value = SoMaterialBinding::PER_PART;
		pcShapeMaterial->diffuseColor.setNum(size);
		pcShapeMaterial->transparency.setNum(size);
		SbColor* ca = pcShapeMaterial->diffuseColor.startEditing();
		float *t = pcShapeMaterial->transparency.startEditing();
		for (int i = 0; i < size; i++) {
			ca[i].setValue(colors[i].r, colors[i].g, colors[i].b);
			t[i] = colors[i].a;
		}
		pcShapeMaterial->diffuseColor.finishEditing();
		pcShapeMaterial->transparency.finishEditing();
	}
	else if (colors.size() == 1) {
		pcFaceBind->value = SoMaterialBinding::OVERALL;
		pcShapeMaterial->diffuseColor.setValue(colors[0].r, colors[0].g, colors[0].b);
		globeColor = colors[0];
		//pcShapeMaterial->transparency = colors[0].a; do not get transparency from DiffuseColor in this case
	}
}

ViewProviderPartExt::~ViewProviderPartExt()
{
	pcFaceBind->unref();
	pcLineBind->unref();
	pcPointBind->unref();
	pcLineMaterial->unref();
	pcPointMaterial->unref();
	pcLineStyle->unref();
	pcPointStyle->unref();
	pShapeHints->unref();
	coords->unref();
	faceset->unref();
	norm->unref();
	normb->unref();
	lineset->unref();
	nodeset->unref();
	pcShapeMaterial->unref();
}

void ViewProviderPartExt::clearColor()
{ 
	pcShapeMaterial->diffuseColor.setValue(0.8, 0.8, 0.8); 
	DiffuseColor.clear(); 
}

void ViewProviderPartExt::clearColor(double r, double g, double b)
{
	pcShapeMaterial->diffuseColor.setValue(r / 255.0, g / 255.0, b / 255.0);
	DiffuseColor.clear();
}

const std::vector<Color> ViewProviderPartExt::getDiffuseColor() const 
{ 
	return DiffuseColor; 
}

const Color ViewProviderPartExt::getGlobeColor() 
{ 
	return globeColor; 
}