#include "types/Mesh.h"

#include <vtksys/SystemTools.hxx>
#include <vtkCellData.h>
#include <vtkTriangle.h>
#include <vtkQuad.h>

#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

const char* GROUPS_TAG_NAME = "group";

namespace meshlib::vtkIO
{

vtkSmartPointer<vtkPolyData> readVTKPolyData(const std::string &fileName)
{
    vtkSmartPointer<vtkPolyData> polyData;
    std::string extension = vtksys::SystemTools::GetFilenameLastExtension(fileName);

    // Drop the case of the extension
    std::transform(extension.begin(), extension.end(), extension.begin(),
                    ::tolower);

    if (extension == ".vtp")
    {
        vtkNew<vtkXMLPolyDataReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".stl")
    {
        vtkNew<vtkSTLReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".vtk")
    {
        vtkNew<vtkPolyDataReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    return polyData;
}

Element vtkCellToElement(vtkCell* cell)
{
    Element elem;
    if (cell->GetCellType() == VTK_TRIANGLE) {
        vtkTriangle* triangle = vtkTriangle::SafeDownCast(cell);
        elem.vertices = {
            CoordinateId(triangle->GetPointIds()->GetId(0)),
            CoordinateId(triangle->GetPointIds()->GetId(1)),
            CoordinateId(triangle->GetPointIds()->GetId(2))
        };
        elem.type = meshlib::Element::Type::Surface;
    }
    return elem;
}

Mesh vtkPolydataToMesh(vtkPolyData* polyData)
{
    Mesh mesh;
    
    mesh.coordinates.reserve(polyData->GetNumberOfPoints());
    for (vtkIdType i = 0; i < polyData->GetNumberOfPoints(); i++)
    {
        double p[3];
        polyData->GetPoint(i, p);
        Coordinate coord({p[0], p[1], p[2]});
        mesh.coordinates.push_back(coord);
    }

    if (polyData->GetCellData()->HasArray(GROUPS_TAG_NAME)) {
        vtkIntArray* groupsDataArray = 
            vtkIntArray::SafeDownCast(polyData->GetCellData()->GetArray(GROUPS_TAG_NAME));
        mesh.groups.resize(groupsDataArray->GetRange()[1] + 1);
        for (vtkIdType i = 0; i < polyData->GetNumberOfCells(); i++) {
            auto g = groupsDataArray->GetValue(i);
            mesh.groups[g].elements.push_back(
                vtkCellToElement(polyData->GetCell(i)));
        }
    } else {
        mesh.groups.resize(1);
        mesh.groups[0].elements.reserve(polyData->GetNumberOfCells());
        for (vtkIdType i = 0; i < polyData->GetNumberOfCells(); i++) {
            mesh.groups[0].elements.push_back(
                vtkCellToElement(polyData->GetCell(i)));
        }
    }

    return mesh;
}

// Assumes coordinates are absolute (not relative to the grid).
vtkSmartPointer<vtkPolyData> meshElementsToVTKPolydata(const Mesh& mesh)
{
    vtkNew<vtkPolyData> polyData;

    vtkNew<vtkPoints> points;
    points->Allocate(mesh.coordinates.size());
    for (const auto& coord : mesh.coordinates) {
        points->InsertNextPoint(coord[0], coord[1], coord[2]);
    }
    polyData->SetPoints(points);

    vtkNew<vtkIntArray> groupsDataArray;
    for (auto g = 0; g < mesh.groups.size(); g++) {
        for (auto e = 0; e < mesh.groups[g].elements.size(); e++) {
            groupsDataArray->InsertNextValue( int(g) );
        }
    }
    groupsDataArray->SetName(GROUPS_TAG_NAME);
    polyData->GetCellData()->AddArray(groupsDataArray);

    vtkNew<vtkCellArray> vtkCells;
    vtkCells->Allocate(mesh.countElems());
    for (const auto& group : mesh.groups) {
        for (const auto& elem : group.elements) {
            vtkNew<vtkTriangle> triangle;
            vtkIdType id = 0;
            for (const auto& vId : elem.vertices) {
                triangle->GetPointIds()->SetId(id++, vId);
            }
            vtkCells->InsertNextCell(triangle);
        }
    }
    polyData->SetPolys(vtkCells);

    return polyData;
}

vtkSmartPointer<vtkPolyData> gridToVTKPolydata(const Grid& grid)
{
    vtkNew<vtkPolyData> polyData;

    using bound = std::array<double,2>;
    std::array<bound,3> bbox = {
        bound{grid[0].front(), grid[0].back()},
        bound{grid[1].front(), grid[1].back()},
        bound{grid[2].front(), grid[2].back()}    
    };

    auto numElements = grid[0].size() + grid[1].size() + grid[2].size();

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> vtkCells;
    points->Allocate(numElements*4);
    vtkCells->Allocate(numElements);
    for (auto x = 0; x < 3; x++) {
        auto y = (x+1)%3;
        auto z = (x+2)%3;
        for (const auto gridLine : grid[x]) {
            vtkIdType ids[4];
            double p[3];
            p[x] = gridLine; 
            p[y] = bbox[y][0]; p[z] = bbox[z][0]; ids[0] = points->InsertNextPoint(p);
            p[y] = bbox[y][1]; p[z] = bbox[z][0]; ids[1] = points->InsertNextPoint(p);
            p[y] = bbox[y][1]; p[z] = bbox[z][1]; ids[2] = points->InsertNextPoint(p);
            p[y] = bbox[y][0]; p[z] = bbox[z][1]; ids[3] = points->InsertNextPoint(p);
            vtkNew<vtkQuad> quad;
            for (auto i = 0; i < 4; i++) {
                quad->GetPointIds()->SetId(i, ids[i]);
            }
            vtkCells->InsertNextCell(quad);
        }
    }
    polyData->SetPoints(points);
    polyData->SetPolys(vtkCells);

    return polyData;
}

Mesh readMesh(const std::string &fileName)
{
    vtkSmartPointer<vtkPolyData> polyData = readVTKPolyData(fileName);
    return vtkPolydataToMesh(polyData);
}

void exportVTKPolyDataToVTP(const std::string& fn, const vtkSmartPointer<vtkPolyData>& polyData)
{
    std::string extension = vtksys::SystemTools::GetFilenameLastExtension(fn);
    if (extension != ".vtp") {
        throw std::runtime_error("Only vtp files are supported for writing");
    }
    vtkNew<vtkXMLPolyDataWriter> writer;
    writer->SetDataModeToAscii();
    writer->SetFileName(fn.c_str());
    writer->SetInputData(polyData);
    writer->Write();
}

void exportMeshToVTP(const std::string& fn, const Mesh& mesh)
{
    exportVTKPolyDataToVTP(fn, meshElementsToVTKPolydata(mesh));
}

void exportGridToVTP(const std::string& fn, const Grid& grid)
{
    exportVTKPolyDataToVTP(fn, gridToVTKPolydata(grid));
}


}
