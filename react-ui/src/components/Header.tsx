import { Container } from "./Container";
import Navbar from "./Navbar";
import { RosStatus } from "./RosStatus";

interface HeaderProps {
    title: string;
    width: string;
    height: string;
    border?: string;
}

function Header({ title, width, height, border }: HeaderProps) {
    return (
        <div style={{ height, width, border }}>
            <h1>{title}</h1>
            <Container title="Container" width="100%" height="30%" border='2px solid black'>
            <RosStatus />
            </Container>
            
            <Navbar />
        </div>
    );
}

export default Header;