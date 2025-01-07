#include "types/Mesh.h"

#include <vtksys/SystemTools.hxx>
#include <vtkTriangle.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>

namespace meshlib
{

    vtkSmartPointer<vtkPolyData> ReadPolyData(const std::string &fileName)
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

    Mesh readVTK(const std::string &fileName)
    {
        vtkSmartPointer<vtkPolyData> polyData = ReadPolyData(fileName);
        Mesh mesh;
        mesh.coordinates.reserve(polyData->GetNumberOfPoints());
        for (vtkIdType i = 0; i < polyData->GetNumberOfPoints(); i++)
        {
            double p[3];
            polyData->GetPoint(i, p);
            Coordinate coord({p[0], p[1], p[2]});
            mesh.coordinates.push_back(coord);
        }
        mesh.groups.resize(1);
        mesh.groups[0].elements.reserve(polyData->GetNumberOfCells());
        for (vtkIdType i = 0; i < polyData->GetNumberOfCells(); i++)
        {
            vtkSmartPointer<vtkCell> cell = polyData->GetCell(i);
            if (cell->GetCellType() == VTK_TRIANGLE)
            {
                vtkSmartPointer<vtkTriangle> triangle = vtkTriangle::SafeDownCast(cell);
                meshlib::Element elem;
                elem.vertices = {
                    CoordinateId(triangle->GetPointIds()->GetId(0)),
                    CoordinateId(triangle->GetPointIds()->GetId(1)),
                    CoordinateId(triangle->GetPointIds()->GetId(2))
                };
                elem.type = meshlib::Element::Type::Surface;
                mesh.groups[0].elements.push_back(elem);
            }
        }
        return mesh;
    }

} // namespace meshlib
