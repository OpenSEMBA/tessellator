#include "Tools.h"

namespace meshlib {
std::pair<IdSet, IdSet> classifyIds(
    const IdSet& ids,
    const std::function<bool(const CoordinateId&)> condition)
{
    IdSet pass;
    std::copy_if(
        ids.begin(), ids.end(),
        std::inserter(pass, pass.begin()),
        condition
    );

    IdSet noPass;
    std::set_difference(
        ids.begin(), ids.end(),
        pass.begin(), pass.end(),
        std::inserter(noPass, noPass.begin())
    );

    return std::make_pair(pass, noPass);
}

IdSet mergeIds(const IdSet& s1, const IdSet& s2)
{
    IdSet r = s1;
    r.insert(s2.begin(), s2.end());
    return r;
}



}