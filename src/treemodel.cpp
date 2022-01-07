
#include <QStringList>
#include <QDebug>

#include <functional>

#include "treemodel.h"
#include "treeitem.h"

/// constructor
TreeModel::TreeModel()
    : QAbstractItemModel{nullptr}
{
    _rootItem = new TreeItem({tr("Title"), tr("Summary")});
}

/// constructor
TreeModel::TreeModel(const QString &data, QObject *parent)
    : QAbstractItemModel{parent}
{
    _rootItem = new TreeItem({tr("Title"), tr("Summary")});
    beginResetModel();
    setupModelData(data.split('\n'), _rootItem);
    endResetModel();
}

/// destructor
TreeModel::~TreeModel()
{
    delete _rootItem;
}

/// get data for the given model index
QVariant TreeModel::data(const QModelIndex &index, int role) const
{
    if(!index.isValid()) return QVariant();

    if(role != Qt::DisplayRole) return QVariant();

    TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
    return item->get_data(index.column());
}
/// flags that enables the item (ItemIsEnabled) and
/// allows it to be selected (ItemIsSelectable)
Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const
{
    if(!index.isValid()) return Qt::NoItemFlags;
    return QAbstractItemModel::flags(index);
}

QVariant TreeModel::headerData(int section, Qt::Orientation orientation,
                               int role) const
{
    if(orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return _rootItem->get_data(section);
    return QVariant();
}
/// get modelitme indexes for the giving item {row, column, parent}
/// views / delegates use it when accessing data
QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if(!hasIndex(row, column, parent)) return QModelIndex();

    TreeItem* parentItem;
    if(!parent.isValid()) parentItem = _rootItem;
    else parentItem = static_cast<TreeItem*>(parent.internalPointer());

    TreeItem* childItem = parentItem->get_childItem(row);
    if(childItem) return createIndex(row, column, childItem);
    return QModelIndex();
}
/// get parent index of a giving item with index
QModelIndex TreeModel::parent(const QModelIndex &index) const
{
    if(!index.isValid()) return QModelIndex();

    TreeItem* childItem = static_cast<TreeItem*>(index.internalPointer());
    TreeItem* parentItem = childItem->get_parentItem();

    if(parentItem == _rootItem) return QModelIndex();
    // set parent column index == 0
    return createIndex(parentItem->get_row(), 0, parentItem);
}
/// get number of child items stored in the TreeItem for a given model index
int TreeModel::rowCount(const QModelIndex &parent) const
{
    // parent column index == 0
    if(parent.column() > 0) return 0;

    TreeItem* parentItem;
    if(!parent.isValid()) parentItem = _rootItem;
    else parentItem = static_cast<TreeItem*>(parent.internalPointer());

    return parentItem->get_childCount();
}
/// get data columns stored in the TreeItem for a given model index
int TreeModel::columnCount(const QModelIndex &parent) const
{
    if(parent.isValid())
        return static_cast<TreeItem*>(parent.internalPointer())->get_columnCount();
    return _rootItem->get_columnCount();
}

void TreeModel::setupModelData(const QStringList &lines, TreeItem *parent)
{
    QVector<TreeItem*> parents;
    QVector<int> indentations;
    parents << parent;
    indentations << 0;

    int number = 0;

    while (number < lines.count()) {
        int position = 0;
        while (position < lines[number].length()) {
            if (lines[number].at(position) != ' ')
                break;
            position++;
        }

        const QString lineData = lines[number].mid(position).trimmed();

        if (!lineData.isEmpty()) {
            // Read the column data from the rest of the line.
            const QStringList columnStrings =
                lineData.split(QLatin1Char('\t'), Qt::SkipEmptyParts);
            QVector<QVariant> columnData;
            columnData.reserve(columnStrings.count());
            for (const QString &columnString : columnStrings)
                columnData << columnString;

            if (position > indentations.last()) {
                // The last child of the current parent is now the new parent
                // unless the current parent has no children.

                if (parents.last()->get_childCount() > 0) {
                    parents << parents.last()->get_childItem(parents.last()->get_childCount()-1);
                    indentations << position;
                }
            } else {
                while (position < indentations.last() && parents.count() > 0) {
                    parents.pop_back();
                    indentations.pop_back();
                }
            }

            // Append a new item to the current parent's list of children.
            parents.last()->appendChild(new TreeItem(columnData, parents.last()));
        }
        ++number;
    }
}
/// set model data using external input
void TreeModel::setupModelData_dfs(const QStringList &lines, TreeItem *parent)
{
    QVector<TreeItem*> parents;
    parents << parent;
    QVector<int> indentations;
    indentations << 0;
    std::function<void(int)> dfs = [&](int lineNo){
        if(lineNo == lines.count()) return;
        int position = 0;
        while (position < lines[lineNo].length()) {
            if (lines[lineNo].at(position) != ' ')
                break;
            position++;
        }
        const QString lineData = lines[lineNo].mid(position).trimmed();
        // Read the column data from the rest of the line.
        const QStringList columnStrings =
            lineData.split(QLatin1Char('\t'), Qt::SkipEmptyParts);
        QVector<QVariant> columnData;
        columnData.reserve(columnStrings.count()*2);
        for (const QString &columnString : columnStrings)
            columnData << columnString << columnString;

        if (!lineData.isEmpty()) {
            if (position > indentations.last()) {
                // The last child of the current parent is now the new parent
                // unless the current parent has no children.
                if (parents.last()->get_childCount() > 0) {
                    parents << parents.last()->get_childItem(parents.last()->get_childCount()-1);
                    indentations << position;
                }
            } else {
                while (position < indentations.last() && parents.count() > 0) {
                    parents.pop_back();
                    indentations.pop_back();
                }
            }
            // Append a new item to the current parent's list of children.
            parents.last()->appendChild(new TreeItem(columnData, parents.last()));
        }
        //if (!lineData.isEmpty()) qDebug() << columnStrings[0] << " " << columnStrings[1] << " " << indentations;
        dfs(lineNo+1);
    };
    dfs(0);
    return;
}

void TreeModel::redraw(const QString &data){
    beginResetModel();
    _rootItem->reset();
    setupModelData(data.split('\n'), _rootItem);
    endResetModel();
}
