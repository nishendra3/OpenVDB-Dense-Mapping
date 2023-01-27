switch (test)
{
    case 1:
        if (test::test1(path))
        {
            std::cout << "Test 1 passed" << std::endl;
        }
        else
        {
            std::cout << "Test 1 failed" << std::endl;
        }
        break;
    case 2:
        if (test::test2(path))
        {
            std::cout << "Test 2 passed" << std::endl;
        }
        else
        {
            std::cout << "Test 2 failed" << std::endl;
        }
        break;
    case 3:
        if (test::test3(path))
        {
            std::cout << "Test 3 passed" << std::endl;
        }
        else
        {
            std::cout << "Test 3 failed" << std::endl;
        }
        break;
    case 4:
        if (test::test4(path))
        {
            std::cout << "Test 4 passed" << std::endl;
        }
        else
        {
            std::cout << "Test 4 failed" << std::endl;
        }
        break;

    default:
        std::cout << "Invalid test number" << std::endl;
        break;
}