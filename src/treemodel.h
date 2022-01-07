#ifndef TREEMODEL_H
#define TREEMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

class TreeItem;

class TreeModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    /// constructor
    explicit TreeModel();
    explicit TreeModel(const QString& data, QObject *parent = nullptr);
    /// destructor
    ~TreeModel();

    /// get data for the given model index
    QVariant data(const QModelIndex &index, int role) const override;
    /// flags that enables the item (ItemIsEnabled) and
    /// allows it to be selected (ItemIsSelectable)
    Qt::ItemFlags flags(const QModelIndex &index) const override;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    /// get modelitme indexes for the giving item {row, column, parent}
    /// views / delegates use it when accessing data
    QModelIndex index(int row, int column,
                      const QModelIndex &parent = QModelIndex()) const override;
    /// get parent index of a giving item with index
    QModelIndex parent(const QModelIndex &index) const override;
    /// get number of child items stored in the TreeItem for a given model index
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    /// get data columns stored in the TreeItem for a given model index
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    /// redraw
    void redraw(const QString &data);
private:
    void setupModelData(const QStringList &lines, TreeItem *parent);
    void setupModelData_dfs(const QStringList &lines, TreeItem *parent);
    TreeItem* _rootItem;
};

#endif // TREEMODEL_H
