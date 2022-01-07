#include "treeitem.h"

/// constructor
TreeItem::TreeItem(const QVector<QVariant> &data, TreeItem *parent)
    : _itemData(data), _parentItem(parent)
{}
/// destructor
TreeItem::~TreeItem()
{
    qDeleteAll(_childItems);
}

/// append child item to current parent item
void TreeItem::appendChild(TreeItem *child)
{
    _childItems.append(child);
}
/// get child item at row index
TreeItem *TreeItem::get_childItem(int row)
{
    if (row < 0 || row >= _childItems.size()) return nullptr;
    return _childItems.at(row);
}
/// get parent item
TreeItem *TreeItem::get_parentItem()
{
    return _parentItem;
}
/// get child count
int TreeItem::get_childCount() const
{
    return _childItems.count();
}
/// get data column count
int TreeItem::get_columnCount() const
{
    return _itemData.count();
}
/// get row number of current item
int TreeItem::get_row() const
{
    if(_parentItem) return _parentItem->_childItems.indexOf(const_cast<TreeItem*>(this));
    else return 0;
}
/// get data at colum index
QVariant TreeItem::get_data(int column) const
{
    if(column < 0 || column >= _itemData.size()) return QVariant();
    return _itemData.at(column);
}
