#include "GridTools.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "CoordGraph.h"
#include "Geometry.h"

namespace meshlib {
namespace utils {


static constexpr double ROUND_FACTOR = 1000000.0;

GridTools::GridTools(const Grid& grid) {
    Grid aux = grid;
    for (Axis d = 0; d < 3; d++) {
        if (grid[d].size() < 2) {
            throw std::runtime_error("Grid must have at least one cell per direction.");
        }

        std::sort(aux[d].begin(), aux[d].end());
        std::size_t lastUsed = 0;
        for (std::size_t i = 0; i < aux[d].size(); i++) {
            if ((i == lastUsed) ||
                !approxDir(aux[d][lastUsed], aux[d][i], 1e-7)) {
                lastUsed = i;
                grid_[d].push_back(aux[d][i]);
            }
        }
    }

}

Grid& GridTools::grid() {
    return grid_;
}

const Grid& GridTools::grid() const {
    return grid_;
}

Cell GridTools::getOffsetWithGrid(const Grid& nG) const
{
    Cell offset;
    for (std::size_t d = 0; d < 3; d++) {
        auto const& mGDir = grid_[d];
        auto const& nGDir = nG[d];
        for (std::size_t i = 0; i < mGDir.size(); i++) {
            if (mGDir[i] == nGDir.front()) {
                offset(d) = (int) i;
            }
        }
    }
    return offset;
}

CellDir GridTools::numCellsDir(const Axis& d) const 
{
    return (CellDir)grid_[d].size() - 1;
}

Cell GridTools::numCells() const {
    Cell res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = numCellsDir(d);
    }
    return res;
}

std::pair<CoordinateDir, CoordinateDir> GridTools::getBoundsDir(
        const Axis& d) const 
{
    return { grid_[d].front(), grid_[d].back() };
}

std::pair<Coordinate, Coordinate> GridTools::getBounds() const 
{
    std::pair<Coordinate, Coordinate> res;
    for (Axis d = 0; d < 3; d++) {
        res.first [d] = grid_[d].front();
        res.second[d] = grid_[d].back();
    }
    return res;
}

CoordinateDir GridTools::getStepDir(const CellDir& cell, const Axis& d) const 
{
    if ((cell < 0) || (cell >= numCellsDir(d)) || (numCellsDir(d) == 0)) {
        return 0.0;
    } else {
        return grid_[d][cell + 1] - grid_[d][cell];
    }
}

std::array<CoordinateDir, 3> GridTools::getStep(const Cell& cell) const 
{
    std::array<CoordinateDir, 3> res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = getStepDir(cell[d], d);
    }
    return res;
}

CoordinateDir GridTools::getStepDir(const RelativeDir& rel,
                                    const Axis& d) const 
{
    return getStepDir(toCellDir(rel), d);
}

std::array<CoordinateDir, 3> GridTools::getStep(const Relative& cell) const 
{
    return getStep(toCell(cell));
}

CoordinateDir GridTools::getPosDir(const RelativeDir& rel,
                                   const Axis& d) const {
    CellDir cell = toCellDir(rel);
    if (cell + 1 <= 0) {
        return grid_[d].front();
    } else if (cell >= numCellsDir(d)) {
        return grid_[d].back();
    }
    return grid_[d][cell] + getStepDir(cell, d)*(rel - cell);
}

Coordinate GridTools::getPos(const Relative& rel) const {
    Coordinate res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = getPosDir(rel[d], d);
    }
    return res;
}

CoordinateDir GridTools::getPosDir(const CellDir& cell,
                                   const Axis& d) const {
    return getPosDir(toRelativeDir(cell), d);
}

Coordinate GridTools::getPos(const Cell& cell) const {
    return getPos(toRelative(cell));
}

CellDir GridTools::getCellDir(const CoordinateDir& pos,
                              const Axis& d) const {
    return toCellDir(getRelativeDir(pos, d));
}

Cell GridTools::getCell(const Coordinate& pos) const {
    return toCell(getRelative(pos));
}

RelativeDir GridTools::getRelativeDir(const CoordinateDir& pos,
                                      const Axis& d) const {
    if (pos < grid_[d].front()) {
        return 0.0;
    }
    if (pos > grid_[d].back()) {
        return (RelativeDir)numCellsDir(d);
    }
    CellDir cellbeg = 0;
    CellDir cellend = numCellsDir(d);
    CellDir cellmed;
    while (cellbeg < cellend) {
        cellmed = cellbeg + (cellend - cellbeg + 1) / 2;
        if (grid_[d][cellmed] <= pos) {
            cellbeg = cellmed;
        }
        else {
            cellend = cellmed - 1;
        }
    }
    return getRelativeDir(pos, d, cellbeg);
}

Relative GridTools::getRelative(const Coordinate& pos) const {
    Relative res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = getRelativeDir(pos[d], d);
    }
    return res;
}

RelativeDir GridTools::getRelativeDir(const CoordinateDir& pos,
                                      const Axis& d,
                                      const CellDir& cell) const {
    if (getStepDir(cell, d) <= 0.0) {
        return cell;
    }
    if (pos == grid_[d][cell]) {
        return cell;
    }
    RelativeDir r = cell + (pos - grid_[d][cell]) / getStepDir(cell, d);
    return std::round(r * ROUND_FACTOR) / ROUND_FACTOR;
}

Relative GridTools::getRelative(const Coordinate& pos,
                                const Cell& cell) const {
    Relative res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = getRelativeDir(pos[d], d, cell[d]);
    }
    return res;
}

CellDir GridTools::toCellDir(const RelativeDir& cell) {
    return (CellDir)std::floor(cell);
}

Cell GridTools::toCell(const Relative& rel) {
    Cell res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = toCellDir(rel[d]);
    }
    return res;
}

RelativeDir GridTools::toRelativeDir(const CellDir& cell) {
    return (RelativeDir)cell;
}

Relative GridTools::toRelative(const Cell& cell) {
    Relative res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = toRelativeDir(cell[d]);
    }
    return res;
}

CellDir GridTools::toNearestVertexDir(const RelativeDir& rel) {
    CellDir res;
    RelativeDir rem;
    res = (CellDir)std::floor(rel);
    rem = RelativeDir(rel - std::floor(rel));
    if (rem >= 0.5) {
        res++;
    }
    return res;
}

Cell GridTools::toNearestVertex(const Relative& rel) {
    Cell res;
    for (Axis d = 0; d < 3; d++) {
        res[d] = toNearestVertexDir(rel[d]);
    }
    return res;
}

Coordinates GridTools::relativeToAbsolute(const Relatives& cs) const
{
    Coordinates rs;
    for (const auto& c : cs) {
        rs.push_back(getPos(c));
    }
    return rs;
}

Relatives GridTools::absoluteToRelative(const Coordinates& cs) const
{
    Coordinates rs;
    for (const auto& c : cs) {
        rs.push_back(getRelative(c));
    }
    return rs;
}

Grid GridTools::getExtendedDualGrid() const
{
    Grid dualGrid;
    Coordinate coord = Coordinate{ { grid_[x][0], grid_[y][0], grid_[z][0]}};
    const Cell cell = getCell(coord);
    std::array < CoordinateDir, 3> cellStep = getStep(cell);
    for (Axis d : {x, y, z}) {
        dualGrid[d].push_back(grid_[d][0] - 0.5 * cellStep[d]);
    }
    
    for (Axis d : {x, y, z}) {
        for (auto it = grid_[d].begin(); it != grid_[d].end()-1; it++) {
            Coordinate coord = Coordinate{ { 0, 0, 0 } };
            coord[d] = *it;
            const Cell cell = getCell(coord);
            std::array < CoordinateDir, 3> cellStep = getStep(cell);
            dualGrid[d].push_back(*it + 0.5 * cellStep[d]);
        }
    }
    
    for (Axis d : {x, y, z}) {
        Coordinate coord = Coordinate{ { 0, 0, 0 } };
        coord[d] = *(grid_[d].end()-2);
        const Cell cell = getCell(coord);
        std::array < CoordinateDir, 3> cellStep = getStep(cell);
        dualGrid[d].push_back(grid_[d].back() + 0.5 * cellStep[d]);
    }

    return dualGrid;
}


bool GridTools::approxDir(const CoordinateDir& lhs,
                          const CoordinateDir& rhs,
                          const CoordinateDir& tol) {
    static const CoordinateDir epsilon   = 1e-11;
    if ((std::abs(lhs) <= epsilon) && (std::abs(rhs) <= epsilon)) {
        return true;
    } else if ((std::abs(lhs) <= epsilon) || (std::abs(rhs) <= epsilon)) {
        return false;
    } else if (std::abs(lhs - rhs) <= tol * std::abs(lhs + rhs)) {
        return true;
    }
    return false;
}

bool GridTools::approx(const Coordinate& lhs,
                       const Coordinate& rhs,
                       const CoordinateDir& tol) {
    for (std::size_t d = 0; d < 3; d++) {
        if (!approxDir(lhs[d], rhs[d], tol)) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<Plane, LinV>> GridTools::getEdgeIntersectionsWithPlanes(
    const TriV& tri) const
{
    std::array<LinV, 3> lines;
    for (std::size_t i = 0; i < 3; i++) {
        lines[i][0] = tri[i];
        lines[i][1] = tri[(i + 1) % 3];
    }
    std::vector<std::pair<Plane, LinV>> res;
    for (std::size_t d = 0; d < 3; d++) {
        for (std::size_t i = 0; i < 3; i++) {
            if (lines[i][0](d) > lines[i][1](d)) {
                std::swap(lines[i][0], lines[i][1]);
            }
        }
        CoordinateDir minPos = lines[0][0](d);
        CoordinateDir maxPos = lines[0][1](d);
        for (std::size_t i = 1; i < 3; i++) {
            if (lines[i][0](d) < minPos) {
                minPos = lines[i][0](d);
            }
            if (lines[i][1](d) > maxPos) {
                maxPos = lines[i][1](d);
            }
        }
        for (std::size_t i = 1; i < 3; i++) {
            if ((lines[i][0](d) == minPos) && (lines[i][1](d) == maxPos)) {
                swap(lines[0], lines[i]);
                break;
            }
        }
        std::array<CellDir, 3> cellIni;
        std::array<CellDir, 3> cellEnd;
        for (std::size_t i = 0; i < 3; i++) {
            cellIni[i] = getCellDir(lines[i][0](d), d) + 1;
            cellEnd[i] = getCellDir(lines[i][1](d), d);
            if (maxPos == getPosDir(cellEnd[i], d)) {
                cellEnd[i]--;
            }
        }
        CoordinateDir t;
        for (std::size_t i = 1; i < 3; i++) {
            for (CellDir cell = cellIni[i]; cell <= cellEnd[i]; cell++) {
                Plane plane = std::make_pair(cell, d);
                LinV intLine;
                CoordinateDir pos = getPosDir(cell, d);
                t = (pos - lines[0][0](d)) / (lines[0][1](d) - lines[0][0](d));
                intLine[0] = (lines[0][1] - lines[0][0]) * t + lines[0][0];
                t = (pos - lines[i][0](d)) / (lines[i][1](d) - lines[i][0](d));
                intLine[1] = (lines[i][1] - lines[i][0]) * t + lines[i][0];
                intLine[0][d] = intLine[1][d] = pos;
                res.push_back(std::make_pair(plane, intLine));
            }
        }
    }

    // Removes lines that are are repeated because they share an edge (corner case at 45 degrees).
    {
        std::multimap<LinV, std::pair<Plane, LinV>> orderedByLin;
        for (auto const& line : res) {
            auto plane = line.first;
            LinV key = line.second;
            std::sort(key.begin(), key.end());
            orderedByLin.emplace(key, line);
        }

        auto approxEqualLinVs = [&](const LinV& a, const LinV& b) {
            return std::equal(a.begin(), a.end(), b.begin(), b.end(),
                [&](const Coordinate& cA, const Coordinate& cB) { return approx(cA, cB); });
        };

        for (auto it = orderedByLin.begin(); it != orderedByLin.end(); ++it) {
            auto itNext = std::next(it);
            if (itNext != orderedByLin.end() && approxEqualLinVs(it->first, itNext->first)) {
                auto lineIt2 = std::find(res.begin(), res.end(), itNext->second);
                res.erase(lineIt2);
            }
        }
    }

    return res;
}


std::set<Cell> GridTools::getTouchingCells(const Relative& v) const 
{
    std::set<Cell> res;
    Cell local = toCell(v);
    for (std::size_t d = 0; d < 3; d++) {
        if (local(d) == numCellsDir(d)) {
            local(d)--;
        }
    }
    res.insert(local);
    
    bool neighInLB[3] = { false, false, false };
    bool neighInUB[3] = { false, false, false };
    for (std::size_t d = 0; d < 3; d++) {
        if (approxDir(round(v(d)) - v(d), 0.0)) {
            bool inLowerBound = approxDir(round(v(d)) - (double)local(d), 0.0);
            bool inUpperBound = approxDir(round(v(d)) - (double)local(d), 1.0);
            bool firstCell = local(d) == 0;
            bool lastCell = local(d) == numCellsDir(d) - 1;
            if (inLowerBound && !firstCell) {
                Cell aux = local;
                aux(d)--;
                res.insert(aux);
                neighInLB[d] = true;
            }
            else if (inUpperBound && !lastCell) {
                Cell aux = local;
                aux(d)++;
                res.insert(aux);
                neighInUB[d] = true;
            }
        }
    }

    if (isRelativeOnCellEdge(v) || isRelativeOnCellCorner(v)) {
        for (std::size_t x = 0; x < 3; x++) {
            std::size_t y = (x + 1) % 3;
            if (neighInLB[x] && neighInLB[y]) {
                Cell aux = local;
                aux(x)--;
                aux(y)--;
                res.insert(aux);
            }
            else if (neighInLB[x] && neighInUB[y]) {
                Cell aux = local;
                aux(x)--;
                aux(y)++;
                res.insert(aux);
            }
            else if (neighInUB[x] && neighInLB[y]) {
                Cell aux = local;
                aux(x)++;
                aux(y)--;
                res.insert(aux);
            }
            else if (neighInUB[x] && neighInUB[y]) {
                Cell aux = local;
                aux(x)++;
                aux(y)++;
                res.insert(aux);
            }
        }
    }
    
    if (isRelativeOnCellCorner(v)) {
        const std::size_t x = 0, y = 1, z = 2;
        Cell aux = local;
        for (std::size_t d = 0; d < 3; d++) {
            if (neighInLB[d]) {
                aux(d)--;
            }
            if (neighInUB[d]) {
                aux(d)++;
            }
        }
        if ((neighInLB[x] || neighInUB[x]) && 
            (neighInLB[y] || neighInUB[y]) &&
            (neighInLB[z] || neighInUB[z])) {
            res.insert(aux);
        }
    }
    
    return res;

}

std::size_t GridTools::countIntersectingPlanes(const Relative& v) {
    std::size_t intersectingPlanes = 0;
    for (Axis d = 0; d < 3; d++) {
        if (approxDir(round(v(d)) - v(d), 0.0)) {
            intersectingPlanes++;
        }
    }
    return intersectingPlanes;

}

bool GridTools::areCoordOnSameFace(const Relative& r1, const Relative& r2) 
{
    if (isRelativeInterior(r1) || isRelativeInterior(r2)) {
        return false;
    }

    std::size_t nEqualCoords = 0;
    for (Axis d = 0; d < 3; d++) {
        if (approxDir(r1(d) - r2(d), 0.0)) {
            nEqualCoords++;
        }
    }

    if (nEqualCoords >= 1){
        return true;
    }
    else {
        return false;
    }
}
bool GridTools::areCoordOnSameEdge(const Relative& r1, const Relative& r2) 
{
    if (isRelativeInterior(r1) || isRelativeInterior(r2)) {
        return false;
    }

    std::size_t nEqualCoords = 0;
    for (Axis d = 0; d < 3; d++) {
        if (approxDir(r1(d) - r2(d), 0.0)) {
            nEqualCoords++;
        }
    }

    if (nEqualCoords >= 2){
        return true;
    }
    else {
        return false;
    }
}

bool GridTools::sameCellProperties(
    const Relative& r1, const Relative& r2) const { 
    
    return isRelativeInterior(    r1) == isRelativeInterior(    r2)
        && isRelativeOnCellCorner(r1) == isRelativeOnCellCorner(r2)
        && isRelativeOnCellEdge(  r1) == isRelativeOnCellEdge(  r2)
        && isRelativeOnCellFace(  r1) == isRelativeOnCellFace(  r2);
}

std::pair<bool,Axis> GridTools::getCellEdgeAxis(const Relative& r) 
{
    bool isOnEdge = isRelativeOnCellEdge(r);
    Axis resAxis;
    for (Axis x = 0; x < 3; x++) {
        Axis y = (x + 1) % 3;
        Axis z = (x + 2) % 3;
        if (approxDir(round(r(x)) - r(x), 0.0) &&
            approxDir(round(r(y)) - r(y), 0.0)) {
            resAxis = z;
            break;
        }
    }
    return std::make_pair(isOnEdge, resAxis);
}

std::pair<bool, Axis> GridTools::getCellFaceAxis(const Relative& r) 
{
    bool isOnFace = isRelativeOnCellFace(r);
    Axis resAxis;
    for (Axis x = 0; x < 3; x++) {
        Axis y = (x + 1) % 3;
        Axis z = (x + 2) % 3;
        if (approxDir(round(r(x)) - r(x), 0.0) &&
            !approxDir(round(r(y)) - r(y), 0.0) && 
            !approxDir(round(r(z)) - r(z), 0.0)) {
            resAxis = x;
            break;
        }
    }
    return std::make_pair(isOnFace, resAxis);
}


std::vector<double> GridTools::linspace(double ini, double end, std::size_t num) {
    double step;
    if (num > 1) {
        step = (end - ini) / (double)(num - 1);
    }
    else {
        step = (end - ini);
    }
    std::vector<double> res(num);
    for (std::size_t i = 0; i < res.size(); i++) {
        if (i == 0) {
            res[i] = ini;
        }
        else {
            res[i] = res[0] + (double) i * step;
        }
    }
    return res;
}

bool GridTools::elementCrossesGrid(const Element& e, const Coordinates& cs) const
{
    if (e.vertices.size() == 0) {
        return false;
    }

    std::map<Cell, std::size_t> timesCellsAreTouched;
    for (auto const& vId : e.vertices) {
        for (auto const& cell : getTouchingCells(cs[vId])) {
            timesCellsAreTouched[cell]++;
        }
    }
    bool crosses = std::all_of(timesCellsAreTouched.begin(), timesCellsAreTouched.end(), 
        [&](auto const& kv) { 
            return kv.second != e.vertices.size(); 
        });
    return crosses;
}


Grid GridTools::buildCartesianGrid(double ini, double end, std::size_t num) {
    Grid grid;
    for (std::size_t d = 0; d < 3; d++) {
        grid[d] = linspace(ini, end, num);
    }
    return grid;
}


bool GridTools::isRelativeInterior(const Relative& v){
    return countIntersectingPlanes(v) == 0;
}

bool GridTools::isRelativeOnCellFace(const Relative& v){
    return countIntersectingPlanes(v) == 1;
}

bool GridTools::isRelativeOnCellEdge(const Relative& v) {
    return countIntersectingPlanes(v) == 2;
}

bool GridTools::isRelativeOnCellCorner(const Relative& v) {
    return countIntersectingPlanes(v) == 3;
}

bool GridTools::isSegmentOnEdge(
    const Relative& r1, 
    const Relative& r2) const {

    if (!areCoordOnSameEdge(r1, r2)) {
        return false;
    }

    if ((isRelativeOnCellEdge(r1) && isRelativeOnCellEdge(r2)) && 
        (getCellEdgeAxis(r1) == getCellEdgeAxis(r2)) &&
        (toCell(r1) == toCell(r2)))
    {
        return true;
    }

    if (isRelativeOnCellCorner(r1) && isRelativeOnCellCorner(r2))
    {
        return true;
    }

    if ((isRelativeOnCellEdge(r1) && isRelativeOnCellCorner(r2)) ||
        (isRelativeOnCellCorner(r1) && isRelativeOnCellEdge(r2)) )
    {
        return true;
    }

    return false;

}

bool GridTools::isSegmentOnFace(
    const Relative& r1,
    const Relative& r2) const {

    if (!areCoordOnSameFace(r1, r2) || areCoordOnSameEdge(r1,r2)) {
        return false;
    }

    if (isRelativeOnCellFace(r1) && isRelativeOnCellFace(r2)) {
        return true;
    }

    if ((isRelativeOnCellFace(r1) && isRelativeOnCellEdge(r2)) &&
        (isRelativeOnCellEdge(r1) && isRelativeOnCellEdge(r2))) {
        return true;
    }

    if ((isRelativeOnCellEdge(r1) && isRelativeOnCellEdge(r2)) &&
        (getCellEdgeAxis(r1) != getCellEdgeAxis(r2)))
    {
        return true;
    }
            
    if ((isRelativeOnCellEdge(r1) && isRelativeOnCellEdge(r2)) &&
        (getCellEdgeAxis(r1) == getCellEdgeAxis(r2)) &&
        (toCell(r1) != toCell(r2)))
    {
        return true;
    }
    
    if (isRelativeOnCellCorner(r1) &&
        (isRelativeOnCellCorner(r2) || isRelativeOnCellEdge(r2) || isRelativeOnCellFace(r2))) {
        return true;
    }

    if (isRelativeOnCellCorner(r2) &&
        (isRelativeOnCellCorner(r1) || isRelativeOnCellEdge(r1) || isRelativeOnCellFace(r1))) {
        return true;
    }


    return false;
}

Axis GridTools::getSegmentAxisOnEdge(
    const Relative& r1,
    const Relative& r2) const {

    if (isRelativeOnCellEdge(r1)) {
        return getCellEdgeAxis(r1).second;
    } 
    else if (isRelativeOnCellEdge(r2)) {
        return getCellEdgeAxis(r2).second;
    }
    else {
        for (Axis d : {0, 1, 2}) {
            if (!approxDir(r1(d) - r2(d), 0.0)) {
                return d;
            }
        }
    } 

    throw std::logic_error(
        "GridTools @ getSegmentAxisOnEdge : segment is not on edge"
    );

}
Axis GridTools::getSegmentAxisOnFace(
    const Relative& r1,
    const Relative& r2) const {

    if (isRelativeOnCellFace(r1)) {
        return getCellFaceAxis(r1).second;
    } 
    else if (isRelativeOnCellFace(r2)) {
        return getCellFaceAxis(r2).second;
    }
    else {
        for (Axis d : {0, 1, 2}) {
            if (approxDir(r1(d) - r2(d), 0.0)) {
                return d;
            }
        }
    } 

    throw std::logic_error(
        "GridTools @ getSegmentAxisOnFace : segment is not on face"
    );

}

std::map<Cell, std::vector<Coordinate*>> GridTools::buildCellCoordMap(
    std::vector<Coordinate>& coords) const 
{
    std::map<Cell, std::vector<Coordinate*>> cells;
    for (auto& c: coords) {
        std::set<Cell> touching = getTouchingCells(c);
        for (auto const& cell : touching) {
            cells[cell].push_back(&c);
        }
    }
    return cells;
}

std::map<Cell, std::vector<const Element*>> GridTools::buildCellElemMap(
    const std::vector<Element>& elems,
    const std::vector<Coordinate>& coords) const
{
    std::map<Cell, std::vector<const Element*>> cells;
    for (auto e = elems.begin(); e != elems.end(); ++e) {
        
        Coordinate centroid;
        for (std::size_t i = 0; i < e->vertices.size(); i++) {
            centroid += coords[e->vertices[i]] / double(e->vertices.size());
        }

        std::set<Cell> touching = getTouchingCells(centroid);
        for (auto const& cell : touching) {
            cells[cell].push_back(&(*e));
        }
    }
    return cells;
}

std::map<Cell, std::vector<const Element*>> GridTools::buildCellTriMap(
    const Elements& elems,
    const Coordinates& coords) const
{
    std::map<Cell, std::vector<const Element*>> cells;
    for (auto e = elems.begin(); e != elems.end(); ++e) {

        if (e->type != Element::Type::Surface) {
            continue;
        }
        Coordinate centroid;
        for (std::size_t i = 0; i < e->vertices.size(); i++) {
            centroid += coords[e->vertices[i]] / double(e->vertices.size());
        }

        std::set<Cell> touching = getTouchingCells(centroid);
        for (auto const& cell : touching) {
            cells[cell].push_back(&(*e));
        }
    }
    return cells;
}



}
}
