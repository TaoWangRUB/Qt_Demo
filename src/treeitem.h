#ifndef TREEITEM_H
#define TREEITEM_H

#include <QVariant>
#include <QVector>

class TreeItem
{
public:
    /// constructor of treeitem with data and parent item
    explicit TreeItem(const QVector<QVariant> &data, TreeItem *parentItem = nullptr);
    ~TreeItem();

    /// append child item to current parent item
    void appendChild(TreeItem *child);
    /// get child item at row index
    TreeItem *get_childItem(int row);
    /// get parent item
    TreeItem *get_parentItem();
    /// get child count
    int get_childCount() const;
    /// get data column count
    int get_columnCount() const;
    /// get data at colum index
    QVariant get_data(int column) const;
    /// get row number of current item
    int get_row() const;
    /// reset
    void reset();

private:
    QVector<TreeItem*> _childItems;
    QVector<QVariant> _itemData;
    TreeItem *_parentItem;
};
//! [0]

#endif // TREEITEM_H
