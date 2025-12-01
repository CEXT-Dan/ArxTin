#pragma once
struct PNEZD
{
    std::variant<size_t, std::wstring> _id;
    AcGePoint3d point;
    std::wstring _description;
};

using PNEZDArray = std::vector<PNEZD>;

bool parse_pnezd(const std::filesystem::path& inpath, char dlm, PNEZDArray& penzdList);