interface SidebarProps {
    width: string;
}

function Sidebar({ width }: SidebarProps) {
    return (
        <div className="sidebar" style={{
            width, border
                : '2px solid black'
        }}>
            <p>Sidebar</p>
        </div>
    );
}

export default Sidebar;