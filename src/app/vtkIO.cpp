#include "vtkIO.h"

#include <vtkCellData.h>
#include <vtkCellType.h>
#include <vtkTriangle.h>
#include <vtkTetra.h>
#include <vtkHexahedron.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkVertex.h>
#include <vtkUnstructuredGrid.h>
#include <vtkStringArray.h>

#include <vtkAppendFilter.h>
#include <vtkSTLReader.h>
#include <vtkUnstructuredGridReader.h>
#include <vtkPolyDataReader.h>

#include <vtkUnstructuredGridWriter.h>

const char* GROUPS_TAG_NAME = "group";

namespace meshlib::vtkIO
{

vtkSmartPointer<vtkUnstructuredGrid> vtkPolyDataToVTU(vtkPolyData* polyData)
{
    vtkNew<vtkAppendFilter> appendFilter;
    appendFilter->AddInputData(polyData);
    appendFilter->Update();
    return appendFilter->GetOutput();
}

vtkSmartPointer<vtkUnstructuredGrid> readAsVTU(const std::filesystem::path& filename)
{
    std::string fn = filename.string();
    // Check if file can be accessed.
    {
        std::ifstream inputStream;
        inputStream.open(fn.c_str(), ios::in);
        if(!inputStream) {
            auto msg = "File could not be opened: " + fn;
            throw std::runtime_error(msg);
        }
    } 

    vtkSmartPointer<vtkUnstructuredGrid> vtu;
    std::string extension = fn.substr(fn.find_last_of(".")).empty() ? "" : fn.substr(fn.find_last_of("."));

    std::transform(extension.begin(), extension.end(), extension.begin(),
                    ::tolower);

    if (extension == ".stl") {
        vtkNew<vtkSTLReader> reader;
        reader->SetFileName(fn.c_str());
        reader->Update();
        vtu = vtkPolyDataToVTU(reader->GetOutput());
    } else if (extension == ".vtk") {
        vtkNew<vtkPolyDataReader> reader;
        reader->SetFileName(fn.c_str());
        reader->Update();
        vtu = vtkPolyDataToVTU(reader->GetOutput());
    } else if (extension == ".vtu") {
        vtkNew<vtkUnstructuredGridReader> reader;
        reader->SetFileName(fn.c_str());
        reader->Update();
        vtu = reader->GetOutput();

    } else {
        throw std::runtime_error("Unsupported file format");
    }
    return vtu;
}

Element vtkCellToElement(vtkCell* cell)
{
    Element elem;
    vtkVertex* vertex = nullptr;
    vtkLine* line = nullptr;
    vtkTriangle* triangle = nullptr;
    vtkTetra* tetra = nullptr;
    vtkHexahedron* hexahedron = nullptr;

    switch (cell->GetCellType()) {
    case VTK_VERTEX:
        vertex = vtkVertex::SafeDownCast(cell);
        elem.vertices = { CoordinateId(vertex->GetPointIds()->GetId(0)) };
        elem.type = meshlib::Element::Type::Node;
        break;
    
    case VTK_LINE:
        line = vtkLine::SafeDownCast(cell);
        elem.vertices = {
            CoordinateId(line->GetPointIds()->GetId(0)),
            CoordinateId(line->GetPointIds()->GetId(1))
        };
        elem.type = meshlib::Element::Type::Line;
        break;

    case VTK_TRIANGLE:
        triangle = vtkTriangle::SafeDownCast(cell);
        elem.vertices = {
            CoordinateId(triangle->GetPointIds()->GetId(0)),
            CoordinateId(triangle->GetPointIds()->GetId(1)),
            CoordinateId(triangle->GetPointIds()->GetId(2))
        };
        elem.type = meshlib::Element::Type::Surface;
        break;

    case VTK_TETRA:
        tetra = vtkTetra::SafeDownCast(cell);
        elem.vertices = {
            CoordinateId(tetra->GetPointIds()->GetId(0)),
            CoordinateId(tetra->GetPointIds()->GetId(1)),
            CoordinateId(tetra->GetPointIds()->GetId(2)),
            CoordinateId(tetra->GetPointIds()->GetId(3))
        };
        elem.type = meshlib::Element::Type::Volume;
        break;

    case VTK_HEXAHEDRON:
        hexahedron = vtkHexahedron::SafeDownCast(cell);
        elem.vertices.reserve(8);
        for (vtkIdType id = 0; id < 8; ++id) {
            elem.vertices.push_back(CoordinateId(hexahedron->GetPointIds()->GetId(id)));
        }
        elem.type = meshlib::Element::Type::Volume;
        break;
    }
    
    return elem;
}

Mesh vtuToMesh(vtkUnstructuredGrid* vtu)
{
    Mesh mesh;
    
    mesh.coordinates.reserve(vtu->GetNumberOfPoints());
    for (vtkIdType i = 0; i < vtu->GetNumberOfPoints(); i++)
    {
        double p[3];
        vtu->GetPoint(i, p);
        Coordinate coord({p[0], p[1], p[2]});
        mesh.coordinates.push_back(coord);
    }

    if (vtu->GetCellData()->HasArray(GROUPS_TAG_NAME)) {
        vtkIntArray* groupsDataArray = 
            vtkIntArray::SafeDownCast(vtu->GetCellData()->GetArray(GROUPS_TAG_NAME));
        vtkStringArray* groupNamesDataArray = vtkStringArray::SafeDownCast(
            vtu->GetCellData()->GetAbstractArray("groupNames"));
        mesh.groups.resize(groupsDataArray->GetRange()[1] + 1);
        for (vtkIdType i = 0; i < vtu->GetNumberOfCells(); i++) {
            auto g = groupsDataArray->GetValue(i);
            mesh.groups[g].elements.push_back(
                vtkCellToElement(vtu->GetCell(i)));
            if (groupNamesDataArray != nullptr && mesh.groups[g].name.empty()) {
                mesh.groups[g].name = groupNamesDataArray->GetValue(i);
            }
        }
    } else {
        mesh.groups.resize(1);
        auto k = vtu->GetNumberOfCells();
        mesh.groups[0].elements.reserve(vtu->GetNumberOfCells());
        for (vtkIdType i = 0; i < vtu->GetNumberOfCells(); i++) {
            mesh.groups[0].elements.push_back(
                vtkCellToElement(vtu->GetCell(i)));
        }
    }

    return mesh;
}

vtkSmartPointer<vtkPoints> toVTKPoints(const std::vector<Coordinate>& coordinates)
{
    vtkNew<vtkPoints> points;
    points->Allocate(coordinates.size());
    for (const auto& coord : coordinates) {
        points->InsertNextPoint(coord[0], coord[1], coord[2]);
    }
    return points;
}   

vtkSmartPointer<vtkIntArray> toVTKGroupsArray(const Mesh& mesh)
{
    vtkNew<vtkIntArray> groupsDataArray;
    for (auto g = 0; g < mesh.groups.size(); g++) {
        for (auto e = 0; e < mesh.groups[g].elements.size(); e++) {
            groupsDataArray->InsertNextValue( int(g) );
        }
    }
    groupsDataArray->SetName(GROUPS_TAG_NAME);
    return groupsDataArray;
}

vtkSmartPointer<vtkStringArray> toVTKGroupNamesArray(const Mesh& mesh)
{
    vtkNew<vtkStringArray> groupNamesArray;
    groupNamesArray->SetName("groupNames");
    groupNamesArray->SetNumberOfComponents(1);
    
    for (const auto& group : mesh.groups) {
        for (std::size_t e = 0; e < group.elements.size(); e++) {
            groupNamesArray->InsertNextValue(group.name.c_str());
        }
    }
    
    return groupNamesArray;
}

vtkSmartPointer<vtkUnstructuredGrid> elementsToVTU(const Mesh& mesh)
{
    vtkNew<vtkUnstructuredGrid> vtu;

    vtu->SetPoints(toVTKPoints(mesh.coordinates));
    vtu->GetCellData()->AddArray(toVTKGroupsArray(mesh));
    vtu->GetCellData()->AddArray(toVTKGroupNamesArray(mesh));

    std::vector<int> cellTypes;
    cellTypes.reserve(mesh.countElems());
    vtkNew<vtkCellArray> vtkCells;
    vtkCells->Allocate(mesh.countElems());
    for (const auto& group : mesh.groups) {
        for (const auto& elem : group.elements) {
            vtkSmartPointer<vtkCell> cell;
            if (elem.isTriangle()) {
                cellTypes.push_back(VTK_TRIANGLE);
                cell = vtkSmartPointer<vtkTriangle>::New();
            } else if (elem.isQuad()) {
                cellTypes.push_back(VTK_QUAD);
                cell = vtkSmartPointer<vtkQuad>::New();
            } else if (elem.isLine()) {
                cellTypes.push_back(VTK_LINE);
                cell = vtkSmartPointer<vtkLine>::New();
            } else if (elem.isNode()) {
                cellTypes.push_back(VTK_VERTEX);
                cell = vtkSmartPointer<vtkVertex>::New();
            } else if (elem.isTetrahedron()) {
                cellTypes.push_back(VTK_TETRA);
                cell = vtkSmartPointer<vtkTetra>::New();
            } else if (elem.isHexahedron()) {
                cellTypes.push_back(VTK_HEXAHEDRON);
                cell = vtkSmartPointer<vtkHexahedron>::New();
            } else {
                throw std::runtime_error("Unsupported element type");
            }
            vtkIdType id = 0;
            for (const auto& vId : elem.vertices) {
                cell->GetPointIds()->SetId(id++, vId);
            }
            vtkCells->InsertNextCell(cell);
        }
    }

    vtu->SetCells(cellTypes.data(), vtkCells);

    return vtu;
}

vtkSmartPointer<vtkUnstructuredGrid> gridToVTU(const Grid& grid)
{
    vtkNew<vtkUnstructuredGrid> vtu;

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
    vtu->SetPoints(points);
    vtu->SetCells(VTK_QUAD, vtkCells);

    return vtu;
}

std::string getBasename(const std::filesystem::path& fn)
{
    return std::filesystem::path(fn).stem().stem().string();
}

std::filesystem::path getFolder(const std::filesystem::path& fn)
{
    return std::filesystem::path(fn).parent_path();
}

Mesh readInputMesh(const std::filesystem::path& filename)
{
    vtkSmartPointer<vtkUnstructuredGrid> vtu = readAsVTU(filename);
    return vtuToMesh(vtu);
}

void exportToVTU(const std::filesystem::path& filename, const vtkSmartPointer<vtkUnstructuredGrid>& vtu)
{
    std::string fn = filename.string();
    vtkNew<vtkUnstructuredGridWriter> writer;
    writer->SetFileName(fn.c_str());
    writer->SetInputData(vtu);
    writer->Write();
}

void exportMeshToVTU(const std::filesystem::path& fn, const Mesh& mesh)
{
    exportToVTU(fn, elementsToVTU(mesh));
}

void exportGridToVTU(const std::filesystem::path& fn, const Grid& grid)
{
    exportToVTU(fn, gridToVTU(grid));
}


}
